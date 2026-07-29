use std::ptr;

use crate::clipper_base::ClipperBase;
use crate::error::{ClipperError, Result};
use crate::helpers::{
    abs, area_out_pt, e2_inserts_before_e1, first_is_bottom_pt, get_bottom_pt, get_overlap,
    horz_segments_overlap, intersect_point, is_horizontal, poly2_contains_poly1,
    pt2_is_between_pt1_and_pt3, reverse_poly_pt_links, slopes_equal_3_points,
    slopes_equal_4_points, swap_poly_indexes, swap_sides, top_x,
};
use crate::types::{
    CInt, ClipType, Direction, EdgeSide, InitOptions, IntPoint, IntersectNode, Join, OutPt, OutRec,
    Path, Paths, PolyFillType, PolyNode, PolyTree, PolyType, SKIP, TEdge, UNASSIGNED,
};

#[derive(Debug)]
pub struct Clipper {
    pub base: ClipperBase,
    pub joins: Vec<Join>,
    pub ghost_joins: Vec<Join>,
    pub intersect_list: Vec<IntersectNode>,
    pub intersect_edges_order: Vec<*mut TEdge>,
    pub clip_type: ClipType,
    pub maxima: Vec<CInt>,
    pub sorted_edges: *mut TEdge,
    pub execute_locked: bool,
    pub clip_fill_type: PolyFillType,
    pub subj_fill_type: PolyFillType,
    pub reverse_output: bool,
    pub using_poly_tree: bool,
    pub strict_simple: bool,
}

impl Default for Clipper {
    fn default() -> Self {
        Self::with_init_options(0)
    }
}

impl Clipper {
    pub fn new() -> Self {
        Self::default()
    }

    // C++: Clipper::Clipper(int initOptions)
    pub fn with_init_options(init_options: i32) -> Self {
        let mut base = ClipperBase::new();
        base.use_full_range = false;
        base.preserve_collinear = (init_options & InitOptions::PreserveCollinear as i32) != 0;
        base.has_open_paths = false;

        Self {
            base,
            joins: Vec::new(),
            ghost_joins: Vec::new(),
            intersect_list: Vec::new(),
            intersect_edges_order: Vec::new(),
            clip_type: ClipType::Intersection,
            maxima: Vec::new(),
            sorted_edges: ptr::null_mut(),
            execute_locked: false,
            clip_fill_type: PolyFillType::EvenOdd,
            subj_fill_type: PolyFillType::EvenOdd,
            reverse_output: (init_options & InitOptions::ReverseSolution as i32) != 0,
            using_poly_tree: false,
            strict_simple: (init_options & InitOptions::StrictlySimple as i32) != 0,
        }
    }

    pub fn reverse_solution(&self) -> bool {
        self.reverse_output
    }

    pub fn set_reverse_solution(&mut self, value: bool) {
        self.reverse_output = value;
    }

    pub fn strictly_simple(&self) -> bool {
        self.strict_simple
    }

    pub fn set_strictly_simple(&mut self, value: bool) {
        self.strict_simple = value;
    }

    pub fn add_path(&mut self, pg: &Path, poly_type: PolyType, closed: bool) -> Result<bool> {
        self.base.add_path(pg, poly_type, closed)
    }

    pub fn add_paths(&mut self, ppg: &Paths, poly_type: PolyType, closed: bool) -> Result<bool> {
        self.base.add_paths(ppg, poly_type, closed)
    }

    // C++: ClipperBase::GetBounds exposed through Clipper
    pub unsafe fn get_bounds(&self) -> crate::types::IntRect {
        unsafe { self.base.get_bounds() }
    }

    // C++: Clipper::Execute(ClipType, Paths&, PolyFillType)
    pub fn execute(
        &mut self,
        clip_type: ClipType,
        solution: &mut Paths,
        fill_type: PolyFillType,
    ) -> Result<bool> {
        self.execute_with_fill_types(clip_type, solution, fill_type, fill_type)
    }

    // C++: Clipper::Execute(ClipType, Paths&, PolyFillType, PolyFillType)
    pub fn execute_with_fill_types(
        &mut self,
        clip_type: ClipType,
        solution: &mut Paths,
        subj_fill_type: PolyFillType,
        clip_fill_type: PolyFillType,
    ) -> Result<bool> {
        if self.execute_locked {
            return Ok(false);
        }
        if self.base.has_open_paths {
            return Err(ClipperError::new(
                "Error: PolyTree struct is needed for open path clipping.",
            ));
        }

        self.execute_locked = true;
        solution.clear();
        self.subj_fill_type = subj_fill_type;
        self.clip_fill_type = clip_fill_type;
        self.clip_type = clip_type;
        self.using_poly_tree = false;

        let result = unsafe { self.execute_internal() };
        if let Ok(true) = result {
            unsafe {
                self.build_result(solution);
            }
        }
        unsafe {
            self.base.dispose_all_out_recs();
        }
        self.execute_locked = false;
        result
    }

    // C++: Clipper::Execute(ClipType, PolyTree&, PolyFillType)
    pub fn execute_polytree(
        &mut self,
        clip_type: ClipType,
        polytree: &mut PolyTree,
        fill_type: PolyFillType,
    ) -> Result<bool> {
        self.execute_polytree_with_fill_types(clip_type, polytree, fill_type, fill_type)
    }

    // C++: Clipper::Execute(ClipType, PolyTree&, PolyFillType, PolyFillType)
    pub fn execute_polytree_with_fill_types(
        &mut self,
        clip_type: ClipType,
        polytree: &mut PolyTree,
        subj_fill_type: PolyFillType,
        clip_fill_type: PolyFillType,
    ) -> Result<bool> {
        if self.execute_locked {
            return Ok(false);
        }

        self.execute_locked = true;
        self.subj_fill_type = subj_fill_type;
        self.clip_fill_type = clip_fill_type;
        self.clip_type = clip_type;
        self.using_poly_tree = true;

        let result = unsafe { self.execute_internal() };
        if let Ok(true) = result {
            unsafe {
                self.build_result2(polytree);
            }
        }
        unsafe {
            self.base.dispose_all_out_recs();
        }
        self.execute_locked = false;
        result
    }

    // C++: Clipper::ExecuteInternal
    pub unsafe fn execute_internal(&mut self) -> Result<bool> {
        let result: Result<bool> = (|| unsafe {
            self.base.reset();
            self.maxima.clear();
            self.sorted_edges = ptr::null_mut();

            let mut bot_y;
            let mut top_y;
            match self.base.pop_scanbeam() {
                Some(y) => bot_y = y,
                None => return Ok(false),
            }
            self.insert_local_minima_into_ael(bot_y)?;
            loop {
                let next_scanbeam = self.base.pop_scanbeam();
                if next_scanbeam.is_none() && !self.base.local_minima_pending() {
                    break;
                }
                top_y = next_scanbeam.unwrap_or(bot_y);
                self.process_horizontals()?;
                self.clear_ghost_joins();
                if !self.process_intersections(top_y)? {
                    return Ok(false);
                }
                self.process_edges_at_top_of_scanbeam(top_y)?;
                bot_y = top_y;
                self.insert_local_minima_into_ael(bot_y)?;
            }

            for outrec in &self.base.poly_outs {
                if (*outrec).is_null() || (*(*outrec)).pts.is_null() || (*(*outrec)).is_open {
                    continue;
                }
                if ((*(*outrec)).is_hole ^ self.reverse_output)
                    == (area_out_pt((*(*outrec)).pts) > 0.0)
                {
                    reverse_poly_pt_links((*(*outrec)).pts);
                }
            }

            if !self.joins.is_empty() {
                self.join_common_edges();
            }

            for i in 0..self.base.poly_outs.len() {
                let outrec = self.base.poly_outs[i];
                if outrec.is_null() || (*outrec).pts.is_null() {
                    continue;
                }
                if (*outrec).is_open {
                    self.fixup_out_polyline(outrec);
                } else {
                    self.fixup_out_polygon(outrec);
                }
            }

            if self.strict_simple {
                self.do_simple_polygons();
            }

            Ok(true)
        })();

        unsafe {
            self.clear_joins();
            self.clear_ghost_joins();
        }
        result
    }

    // C++: Clipper::FixHoleLinkage
    pub unsafe fn fix_hole_linkage(&mut self, outrec: *mut crate::types::OutRec) {
        unsafe {
            if (*outrec).first_left.is_null()
                || ((*outrec).is_hole != (*(*outrec).first_left).is_hole
                    && !(*(*outrec).first_left).pts.is_null())
            {
                return;
            }

            let mut orfl = (*outrec).first_left;
            while !orfl.is_null() && ((*orfl).is_hole == (*outrec).is_hole || (*orfl).pts.is_null())
            {
                orfl = (*orfl).first_left;
            }
            (*outrec).first_left = orfl;
        }
    }

    // C++: Clipper::AddLocalMinPoly
    pub unsafe fn add_local_min_poly(
        &mut self,
        e1: *mut TEdge,
        e2: *mut TEdge,
        pt: IntPoint,
    ) -> *mut OutPt {
        unsafe {
            let (result, e, prev_e);
            if is_horizontal(&*e2) || (*e1).dx > (*e2).dx {
                result = self.add_out_pt(e1, pt);
                (*e2).out_idx = (*e1).out_idx;
                (*e1).side = EdgeSide::Left;
                (*e2).side = EdgeSide::Right;
                e = e1;
                prev_e = if (*e).prev_in_ael == e2 {
                    (*e2).prev_in_ael
                } else {
                    (*e).prev_in_ael
                };
            } else {
                result = self.add_out_pt(e2, pt);
                (*e1).out_idx = (*e2).out_idx;
                (*e1).side = EdgeSide::Right;
                (*e2).side = EdgeSide::Left;
                e = e2;
                prev_e = if (*e).prev_in_ael == e1 {
                    (*e1).prev_in_ael
                } else {
                    (*e).prev_in_ael
                };
            }

            if !prev_e.is_null()
                && (*prev_e).out_idx >= 0
                && (*prev_e).top.y < pt.y
                && (*e).top.y < pt.y
            {
                let x_prev = top_x(&*prev_e, pt.y);
                let x_e = top_x(&*e, pt.y);
                if x_prev == x_e
                    && (*e).wind_delta != 0
                    && (*prev_e).wind_delta != 0
                    && slopes_equal_4_points(
                        IntPoint::new(x_prev, pt.y),
                        (*prev_e).top,
                        IntPoint::new(x_e, pt.y),
                        (*e).top,
                        self.base.use_full_range,
                    )
                {
                    let out_pt = self.add_out_pt(prev_e, pt);
                    self.add_join(result, out_pt, (*e).top);
                }
            }
            result
        }
    }

    // C++: Clipper::AddLocalMaxPoly
    pub unsafe fn add_local_max_poly(&mut self, e1: *mut TEdge, e2: *mut TEdge, pt: IntPoint) {
        unsafe {
            self.add_out_pt(e1, pt);
            if (*e2).wind_delta == 0 {
                self.add_out_pt(e2, pt);
            }
            if (*e1).out_idx == (*e2).out_idx {
                (*e1).out_idx = UNASSIGNED;
                (*e2).out_idx = UNASSIGNED;
            } else if (*e1).out_idx < (*e2).out_idx {
                self.append_polygon(e1, e2);
            } else {
                self.append_polygon(e2, e1);
            }
        }
    }

    // C++: Clipper::SetHoleState
    pub unsafe fn set_hole_state(&mut self, e: *mut TEdge, outrec: *mut OutRec) {
        unsafe {
            let mut e2 = (*e).prev_in_ael;
            let mut e_tmp: *mut TEdge = ptr::null_mut();
            while !e2.is_null() {
                if (*e2).out_idx >= 0 && (*e2).wind_delta != 0 {
                    if e_tmp.is_null() {
                        e_tmp = e2;
                    } else if (*e_tmp).out_idx == (*e2).out_idx {
                        e_tmp = ptr::null_mut();
                    }
                }
                e2 = (*e2).prev_in_ael;
            }
            if e_tmp.is_null() {
                (*outrec).first_left = ptr::null_mut();
                (*outrec).is_hole = false;
            } else {
                (*outrec).first_left = self.base.poly_outs[(*e_tmp).out_idx as usize];
                (*outrec).is_hole = !(*(*outrec).first_left).is_hole;
            }
        }
    }

    // C++: Clipper::GetOutRec
    pub unsafe fn get_out_rec(&self, idx: i32) -> *mut OutRec {
        unsafe {
            let mut outrec = self.base.poly_outs[idx as usize];
            while outrec != self.base.poly_outs[(*outrec).idx as usize] {
                outrec = self.base.poly_outs[(*outrec).idx as usize];
            }
            outrec
        }
    }

    // C++: Clipper::AppendPolygon
    pub unsafe fn append_polygon(&mut self, e1: *mut TEdge, e2: *mut TEdge) {
        unsafe {
            let out_rec1 = self.base.poly_outs[(*e1).out_idx as usize];
            let out_rec2 = self.base.poly_outs[(*e2).out_idx as usize];

            let hole_state_rec = if out_rec1_right_of_out_rec2(out_rec1, out_rec2) {
                out_rec2
            } else if out_rec1_right_of_out_rec2(out_rec2, out_rec1) {
                out_rec1
            } else {
                get_lowermost_rec(out_rec1, out_rec2)
            };

            let p1_lft = (*out_rec1).pts;
            let p1_rt = (*p1_lft).prev;
            let p2_lft = (*out_rec2).pts;
            let p2_rt = (*p2_lft).prev;

            if (*e1).side == EdgeSide::Left {
                if (*e2).side == EdgeSide::Left {
                    reverse_poly_pt_links(p2_lft);
                    (*p2_lft).next = p1_lft;
                    (*p1_lft).prev = p2_lft;
                    (*p1_rt).next = p2_rt;
                    (*p2_rt).prev = p1_rt;
                    (*out_rec1).pts = p2_rt;
                } else {
                    (*p2_rt).next = p1_lft;
                    (*p1_lft).prev = p2_rt;
                    (*p2_lft).prev = p1_rt;
                    (*p1_rt).next = p2_lft;
                    (*out_rec1).pts = p2_lft;
                }
            } else if (*e2).side == EdgeSide::Right {
                reverse_poly_pt_links(p2_lft);
                (*p1_rt).next = p2_rt;
                (*p2_rt).prev = p1_rt;
                (*p2_lft).next = p1_lft;
                (*p1_lft).prev = p2_lft;
            } else {
                (*p1_rt).next = p2_lft;
                (*p2_lft).prev = p1_rt;
                (*p1_lft).prev = p2_rt;
                (*p2_rt).next = p1_lft;
            }

            (*out_rec1).bottom_pt = ptr::null_mut();
            if hole_state_rec == out_rec2 {
                if (*out_rec2).first_left != out_rec1 {
                    (*out_rec1).first_left = (*out_rec2).first_left;
                }
                (*out_rec1).is_hole = (*out_rec2).is_hole;
            }
            (*out_rec2).pts = ptr::null_mut();
            (*out_rec2).bottom_pt = ptr::null_mut();
            (*out_rec2).first_left = out_rec1;

            let ok_idx = (*e1).out_idx;
            let obsolete_idx = (*e2).out_idx;

            (*e1).out_idx = UNASSIGNED;
            (*e2).out_idx = UNASSIGNED;

            let mut e = self.base.active_edges;
            while !e.is_null() {
                if (*e).out_idx == obsolete_idx {
                    (*e).out_idx = ok_idx;
                    (*e).side = (*e1).side;
                    break;
                }
                e = (*e).next_in_ael;
            }

            (*out_rec2).idx = (*out_rec1).idx;
        }
    }

    // C++: Clipper::AddOutPt
    pub unsafe fn add_out_pt(&mut self, e: *mut TEdge, pt: IntPoint) -> *mut OutPt {
        unsafe {
            if (*e).out_idx < 0 {
                let out_rec = self.base.create_out_rec();
                (*out_rec).is_open = (*e).wind_delta == 0;
                let new_op = self.base.create_out_pt(OutPt {
                    idx: (*out_rec).idx,
                    pt,
                    next: ptr::null_mut(),
                    prev: ptr::null_mut(),
                });
                (*out_rec).pts = new_op;
                (*new_op).next = new_op;
                (*new_op).prev = new_op;
                if !(*out_rec).is_open {
                    self.set_hole_state(e, out_rec);
                }
                (*e).out_idx = (*out_rec).idx;
                new_op
            } else {
                let out_rec = self.base.poly_outs[(*e).out_idx as usize];
                let op = (*out_rec).pts;

                let to_front = (*e).side == EdgeSide::Left;
                if to_front && pt == (*op).pt {
                    return op;
                } else if !to_front && pt == (*(*op).prev).pt {
                    return (*op).prev;
                }

                let new_op = self.base.create_out_pt(OutPt {
                    idx: (*out_rec).idx,
                    pt,
                    next: op,
                    prev: (*op).prev,
                });
                (*(*new_op).prev).next = new_op;
                (*op).prev = new_op;
                if to_front {
                    (*out_rec).pts = new_op;
                }
                new_op
            }
        }
    }

    // C++: Clipper::GetLastOutPt
    pub unsafe fn get_last_out_pt(&self, e: *mut TEdge) -> *mut OutPt {
        unsafe {
            let out_rec = self.base.poly_outs[(*e).out_idx as usize];
            if (*e).side == EdgeSide::Left {
                (*out_rec).pts
            } else {
                (*(*out_rec).pts).prev
            }
        }
    }

    // C++: Clipper::InsertLocalMinimaIntoAEL
    pub unsafe fn insert_local_minima_into_ael(&mut self, bot_y: CInt) -> Result<()> {
        while let Some(lm) = self.base.pop_local_minima(bot_y) {
            unsafe {
                let lb = lm.left_bound;
                let rb = lm.right_bound;
                let mut op1: *mut OutPt = ptr::null_mut();

                if lb.is_null() {
                    self.insert_edge_into_ael(rb, ptr::null_mut());
                    self.set_winding_count(rb);
                    if self.is_contributing(&*rb) {
                        op1 = self.add_out_pt(rb, (*rb).bot);
                    }
                } else if rb.is_null() {
                    self.insert_edge_into_ael(lb, ptr::null_mut());
                    self.set_winding_count(lb);
                    if self.is_contributing(&*lb) {
                        op1 = self.add_out_pt(lb, (*lb).bot);
                    }
                    self.base.insert_scanbeam((*lb).top.y);
                } else {
                    self.insert_edge_into_ael(lb, ptr::null_mut());
                    self.insert_edge_into_ael(rb, lb);
                    self.set_winding_count(lb);
                    (*rb).wind_cnt = (*lb).wind_cnt;
                    (*rb).wind_cnt2 = (*lb).wind_cnt2;
                    if self.is_contributing(&*lb) {
                        op1 = self.add_local_min_poly(lb, rb, (*lb).bot);
                    }
                    self.base.insert_scanbeam((*lb).top.y);
                }

                if !rb.is_null() {
                    if is_horizontal(&*rb) {
                        self.add_edge_to_sel(rb);
                        if !(*rb).next_in_lml.is_null() {
                            self.base.insert_scanbeam((*(*rb).next_in_lml).top.y);
                        }
                    } else {
                        self.base.insert_scanbeam((*rb).top.y);
                    }
                }

                if lb.is_null() || rb.is_null() {
                    continue;
                }

                if !op1.is_null()
                    && is_horizontal(&*rb)
                    && !self.ghost_joins.is_empty()
                    && (*rb).wind_delta != 0
                {
                    for i in 0..self.ghost_joins.len() {
                        let jr = self.ghost_joins[i];
                        if horz_segments_overlap(
                            (*jr.out_pt1).pt.x,
                            jr.off_pt.x,
                            (*rb).bot.x,
                            (*rb).top.x,
                        ) {
                            self.add_join(jr.out_pt1, op1, jr.off_pt);
                        }
                    }
                }

                if (*lb).out_idx >= 0
                    && !(*lb).prev_in_ael.is_null()
                    && (*(*lb).prev_in_ael).curr.x == (*lb).bot.x
                    && (*(*lb).prev_in_ael).out_idx >= 0
                    && slopes_equal_4_points(
                        (*(*lb).prev_in_ael).bot,
                        (*(*lb).prev_in_ael).top,
                        (*lb).curr,
                        (*lb).top,
                        self.base.use_full_range,
                    )
                    && (*lb).wind_delta != 0
                    && (*(*lb).prev_in_ael).wind_delta != 0
                {
                    let op2 = self.add_out_pt((*lb).prev_in_ael, (*lb).bot);
                    self.add_join(op1, op2, (*lb).top);
                }

                if (*lb).next_in_ael != rb {
                    if (*rb).out_idx >= 0
                        && !(*rb).prev_in_ael.is_null()
                        && (*(*rb).prev_in_ael).out_idx >= 0
                        && slopes_equal_4_points(
                            (*(*rb).prev_in_ael).curr,
                            (*(*rb).prev_in_ael).top,
                            (*rb).curr,
                            (*rb).top,
                            self.base.use_full_range,
                        )
                        && (*rb).wind_delta != 0
                        && (*(*rb).prev_in_ael).wind_delta != 0
                    {
                        let op2 = self.add_out_pt((*rb).prev_in_ael, (*rb).bot);
                        self.add_join(op1, op2, (*rb).top);
                    }

                    let e = (*lb).next_in_ael;
                    if !e.is_null() {
                        let mut e = e;
                        while e != rb {
                            self.intersect_edges(rb, e, (*lb).curr);
                            e = (*e).next_in_ael;
                        }
                    }
                }
            }
        }
        Ok(())
    }

    // C++: Clipper::AddEdgeToSEL
    pub unsafe fn add_edge_to_sel(&mut self, edge: *mut TEdge) {
        unsafe {
            if self.sorted_edges.is_null() {
                self.sorted_edges = edge;
                (*edge).prev_in_sel = ptr::null_mut();
                (*edge).next_in_sel = ptr::null_mut();
            } else {
                (*edge).next_in_sel = self.sorted_edges;
                (*edge).prev_in_sel = ptr::null_mut();
                (*self.sorted_edges).prev_in_sel = edge;
                self.sorted_edges = edge;
            }
        }
    }

    // C++: Clipper::PopEdgeFromSEL
    pub unsafe fn pop_edge_from_sel(&mut self) -> Option<*mut TEdge> {
        if self.sorted_edges.is_null() {
            return None;
        }
        let edge = self.sorted_edges;
        unsafe {
            self.delete_from_sel(self.sorted_edges);
        }
        Some(edge)
    }

    // C++: Clipper::CopyAELToSEL
    pub unsafe fn copy_ael_to_sel(&mut self) {
        unsafe {
            let mut e = self.base.active_edges;
            self.sorted_edges = e;
            while !e.is_null() {
                (*e).prev_in_sel = (*e).prev_in_ael;
                (*e).next_in_sel = (*e).next_in_ael;
                e = (*e).next_in_ael;
            }
        }
    }

    // C++: Clipper::DeleteFromSEL
    pub unsafe fn delete_from_sel(&mut self, e: *mut TEdge) {
        unsafe {
            let sel_prev = (*e).prev_in_sel;
            let sel_next = (*e).next_in_sel;
            if sel_prev.is_null() && sel_next.is_null() && e != self.sorted_edges {
                return;
            }
            if !sel_prev.is_null() {
                (*sel_prev).next_in_sel = sel_next;
            } else {
                self.sorted_edges = sel_next;
            }
            if !sel_next.is_null() {
                (*sel_next).prev_in_sel = sel_prev;
            }
            (*e).next_in_sel = ptr::null_mut();
            (*e).prev_in_sel = ptr::null_mut();
        }
    }

    // C++: Clipper::SwapPositionsInSEL
    pub unsafe fn swap_positions_in_sel(&mut self, edge1: *mut TEdge, edge2: *mut TEdge) {
        unsafe {
            if (*edge1).next_in_sel.is_null() && (*edge1).prev_in_sel.is_null() {
                return;
            }
            if (*edge2).next_in_sel.is_null() && (*edge2).prev_in_sel.is_null() {
                return;
            }

            if (*edge1).next_in_sel == edge2 {
                let next = (*edge2).next_in_sel;
                if !next.is_null() {
                    (*next).prev_in_sel = edge1;
                }
                let prev = (*edge1).prev_in_sel;
                if !prev.is_null() {
                    (*prev).next_in_sel = edge2;
                }
                (*edge2).prev_in_sel = prev;
                (*edge2).next_in_sel = edge1;
                (*edge1).prev_in_sel = edge2;
                (*edge1).next_in_sel = next;
            } else if (*edge2).next_in_sel == edge1 {
                let next = (*edge1).next_in_sel;
                if !next.is_null() {
                    (*next).prev_in_sel = edge2;
                }
                let prev = (*edge2).prev_in_sel;
                if !prev.is_null() {
                    (*prev).next_in_sel = edge1;
                }
                (*edge1).prev_in_sel = prev;
                (*edge1).next_in_sel = edge2;
                (*edge2).prev_in_sel = edge1;
                (*edge2).next_in_sel = next;
            } else {
                let next = (*edge1).next_in_sel;
                let prev = (*edge1).prev_in_sel;
                (*edge1).next_in_sel = (*edge2).next_in_sel;
                if !(*edge1).next_in_sel.is_null() {
                    (*(*edge1).next_in_sel).prev_in_sel = edge1;
                }
                (*edge1).prev_in_sel = (*edge2).prev_in_sel;
                if !(*edge1).prev_in_sel.is_null() {
                    (*(*edge1).prev_in_sel).next_in_sel = edge1;
                }
                (*edge2).next_in_sel = next;
                if !(*edge2).next_in_sel.is_null() {
                    (*(*edge2).next_in_sel).prev_in_sel = edge2;
                }
                (*edge2).prev_in_sel = prev;
                if !(*edge2).prev_in_sel.is_null() {
                    (*(*edge2).prev_in_sel).next_in_sel = edge2;
                }
            }

            if (*edge1).prev_in_sel.is_null() {
                self.sorted_edges = edge1;
            } else if (*edge2).prev_in_sel.is_null() {
                self.sorted_edges = edge2;
            }
        }
    }

    // C++: Clipper::InsertEdgeIntoAEL
    pub unsafe fn insert_edge_into_ael(&mut self, edge: *mut TEdge, mut start_edge: *mut TEdge) {
        unsafe {
            if self.base.active_edges.is_null() {
                (*edge).prev_in_ael = ptr::null_mut();
                (*edge).next_in_ael = ptr::null_mut();
                self.base.active_edges = edge;
            } else if start_edge.is_null() && e2_inserts_before_e1(&*self.base.active_edges, &*edge)
            {
                (*edge).prev_in_ael = ptr::null_mut();
                (*edge).next_in_ael = self.base.active_edges;
                (*self.base.active_edges).prev_in_ael = edge;
                self.base.active_edges = edge;
            } else {
                if start_edge.is_null() {
                    start_edge = self.base.active_edges;
                }
                while !(*start_edge).next_in_ael.is_null()
                    && !e2_inserts_before_e1(&*(*start_edge).next_in_ael, &*edge)
                {
                    start_edge = (*start_edge).next_in_ael;
                }
                (*edge).next_in_ael = (*start_edge).next_in_ael;
                if !(*start_edge).next_in_ael.is_null() {
                    (*(*start_edge).next_in_ael).prev_in_ael = edge;
                }
                (*edge).prev_in_ael = start_edge;
                (*start_edge).next_in_ael = edge;
            }
        }
    }

    // C++: Clipper::BuildIntersectList
    pub unsafe fn build_intersect_list(&mut self, top_y: CInt) {
        if self.base.active_edges.is_null() {
            return;
        }

        unsafe {
            let mut e = self.base.active_edges;
            self.intersect_edges_order.clear();
            while !e.is_null() {
                (*e).curr.x = top_x(&*e, top_y);
                self.intersect_edges_order.push(e);
                e = (*e).next_in_ael;
            }
            self.intersect_list
                .reserve(self.intersect_edges_order.len() / 2);

            let mut unsorted_end = self.intersect_edges_order.len();
            while unsorted_end > 1 {
                let mut is_modified = false;
                let mut i = 0usize;
                while i + 1 < unsorted_end {
                    let edge = *self.intersect_edges_order.get_unchecked(i);
                    let edge_next = *self.intersect_edges_order.get_unchecked(i + 1);
                    if (*edge).curr.x > (*edge_next).curr.x {
                        let mut pt = IntPoint::default();
                        intersect_point(&*edge, &*edge_next, &mut pt);
                        if pt.y < top_y {
                            pt = IntPoint::new(top_x(&*edge, top_y), top_y);
                        }
                        self.intersect_list.push(IntersectNode {
                            edge1: edge,
                            edge2: edge_next,
                            pt,
                        });

                        self.intersect_edges_order.swap(i, i + 1);
                        is_modified = true;
                    }
                    i += 1;
                }
                if !is_modified {
                    break;
                }
                unsorted_end -= 1;
            }
            self.sorted_edges = ptr::null_mut();
        }
    }

    // C++: Clipper::ProcessIntersections
    pub unsafe fn process_intersections(&mut self, top_y: CInt) -> Result<bool> {
        if self.base.active_edges.is_null() {
            return Ok(true);
        }

        unsafe {
            self.build_intersect_list(top_y);
            let il_size = self.intersect_list.len();
            if il_size == 0 {
                return Ok(true);
            }
            if il_size == 1 || self.fixup_intersection_order() {
                self.process_intersect_list();
            } else {
                return Ok(false);
            }
            self.sorted_edges = ptr::null_mut();
        }
        Ok(true)
    }

    // C++: Clipper::ProcessHorizontals
    pub unsafe fn process_horizontals(&mut self) -> Result<()> {
        while let Some(horz_edge) = unsafe { self.pop_edge_from_sel() } {
            unsafe {
                self.process_horizontal(horz_edge)?;
            }
        }
        Ok(())
    }

    // C++: Clipper::ProcessHorizontal
    pub unsafe fn process_horizontal(&mut self, horz_edge: *mut TEdge) -> Result<()> {
        unsafe {
            let mut horz_edge = horz_edge;
            let mut is_open = (*horz_edge).wind_delta == 0;

            let (mut dir, mut horz_left, mut horz_right) = get_horz_direction(&*horz_edge);

            let mut e_last_horz = horz_edge;
            let mut e_max_pair: *mut TEdge = ptr::null_mut();
            while !(*e_last_horz).next_in_lml.is_null()
                && is_horizontal(&*(*e_last_horz).next_in_lml)
            {
                e_last_horz = (*e_last_horz).next_in_lml;
            }
            if (*e_last_horz).next_in_lml.is_null() {
                e_max_pair = get_maxima_pair(e_last_horz);
            }

            let mut max_it = 0usize;
            let mut max_rit = self.maxima.len();
            if !self.maxima.is_empty() {
                if dir == Direction::LeftToRight {
                    while max_it < self.maxima.len() && self.maxima[max_it] <= (*horz_edge).bot.x {
                        max_it += 1;
                    }
                    if max_it < self.maxima.len() && self.maxima[max_it] >= (*e_last_horz).top.x {
                        max_it = self.maxima.len();
                    }
                } else {
                    while max_rit > 0 && self.maxima[max_rit - 1] > (*horz_edge).bot.x {
                        max_rit -= 1;
                    }
                    if max_rit > 0 && self.maxima[max_rit - 1] <= (*e_last_horz).top.x {
                        max_rit = 0;
                    }
                }
            }

            let mut op1: *mut OutPt = ptr::null_mut();

            loop {
                let is_last_horz = horz_edge == e_last_horz;
                let mut e = get_next_in_ael(horz_edge, dir);
                while !e.is_null() {
                    if !self.maxima.is_empty() {
                        if dir == Direction::LeftToRight {
                            while max_it < self.maxima.len() && self.maxima[max_it] < (*e).curr.x {
                                if (*horz_edge).out_idx >= 0 && !is_open {
                                    self.add_out_pt(
                                        horz_edge,
                                        IntPoint::new(self.maxima[max_it], (*horz_edge).bot.y),
                                    );
                                }
                                max_it += 1;
                            }
                        } else {
                            while max_rit > 0 && self.maxima[max_rit - 1] > (*e).curr.x {
                                if (*horz_edge).out_idx >= 0 && !is_open {
                                    self.add_out_pt(
                                        horz_edge,
                                        IntPoint::new(self.maxima[max_rit - 1], (*horz_edge).bot.y),
                                    );
                                }
                                max_rit -= 1;
                            }
                        }
                    }

                    if (dir == Direction::LeftToRight && (*e).curr.x > horz_right)
                        || (dir == Direction::RightToLeft && (*e).curr.x < horz_left)
                    {
                        break;
                    }

                    if (*e).curr.x == (*horz_edge).top.x
                        && !(*horz_edge).next_in_lml.is_null()
                        && (*e).dx < (*(*horz_edge).next_in_lml).dx
                    {
                        break;
                    }

                    if (*horz_edge).out_idx >= 0 && !is_open {
                        op1 = self.add_out_pt(horz_edge, (*e).curr);
                        let mut e_next_horz = self.sorted_edges;
                        while !e_next_horz.is_null() {
                            if (*e_next_horz).out_idx >= 0
                                && horz_segments_overlap(
                                    (*horz_edge).bot.x,
                                    (*horz_edge).top.x,
                                    (*e_next_horz).bot.x,
                                    (*e_next_horz).top.x,
                                )
                            {
                                let op2 = self.get_last_out_pt(e_next_horz);
                                self.add_join(op2, op1, (*e_next_horz).top);
                            }
                            e_next_horz = (*e_next_horz).next_in_sel;
                        }
                        self.add_ghost_join(op1, (*horz_edge).bot);
                    }

                    if e == e_max_pair && is_last_horz {
                        if (*horz_edge).out_idx >= 0 {
                            self.add_local_max_poly(horz_edge, e_max_pair, (*horz_edge).top);
                        }
                        self.base.delete_from_ael(horz_edge);
                        self.base.delete_from_ael(e_max_pair);
                        return Ok(());
                    }

                    let pt = IntPoint::new((*e).curr.x, (*horz_edge).curr.y);
                    if dir == Direction::LeftToRight {
                        self.intersect_edges(horz_edge, e, pt);
                    } else {
                        self.intersect_edges(e, horz_edge, pt);
                    }
                    let e_next = get_next_in_ael(e, dir);
                    self.base.swap_positions_in_ael(horz_edge, e);
                    e = e_next;
                }

                if (*horz_edge).next_in_lml.is_null() || !is_horizontal(&*(*horz_edge).next_in_lml)
                {
                    break;
                }

                self.base.update_edge_into_ael(&mut horz_edge)?;
                is_open = (*horz_edge).wind_delta == 0;
                if (*horz_edge).out_idx >= 0 {
                    self.add_out_pt(horz_edge, (*horz_edge).bot);
                }
                (dir, horz_left, horz_right) = get_horz_direction(&*horz_edge);
            }

            if (*horz_edge).out_idx >= 0 && op1.is_null() {
                op1 = self.get_last_out_pt(horz_edge);
                let mut e_next_horz = self.sorted_edges;
                while !e_next_horz.is_null() {
                    if (*e_next_horz).out_idx >= 0
                        && horz_segments_overlap(
                            (*horz_edge).bot.x,
                            (*horz_edge).top.x,
                            (*e_next_horz).bot.x,
                            (*e_next_horz).top.x,
                        )
                    {
                        let op2 = self.get_last_out_pt(e_next_horz);
                        self.add_join(op2, op1, (*e_next_horz).top);
                    }
                    e_next_horz = (*e_next_horz).next_in_sel;
                }
                self.add_ghost_join(op1, (*horz_edge).top);
            }

            if !(*horz_edge).next_in_lml.is_null() {
                if (*horz_edge).out_idx >= 0 {
                    op1 = self.add_out_pt(horz_edge, (*horz_edge).top);
                    self.base.update_edge_into_ael(&mut horz_edge)?;
                    if (*horz_edge).wind_delta == 0 {
                        return Ok(());
                    }

                    let e_prev = (*horz_edge).prev_in_ael;
                    let e_next = (*horz_edge).next_in_ael;
                    if !e_prev.is_null()
                        && (*e_prev).curr.x == (*horz_edge).bot.x
                        && (*e_prev).curr.y == (*horz_edge).bot.y
                        && (*e_prev).wind_delta != 0
                        && (*e_prev).out_idx >= 0
                        && (*e_prev).curr.y > (*e_prev).top.y
                        && slopes_equal_4_points(
                            (*horz_edge).bot,
                            (*horz_edge).top,
                            (*e_prev).bot,
                            (*e_prev).top,
                            self.base.use_full_range,
                        )
                    {
                        let op2 = self.add_out_pt(e_prev, (*horz_edge).bot);
                        self.add_join(op1, op2, (*horz_edge).top);
                    } else if !e_next.is_null()
                        && (*e_next).curr.x == (*horz_edge).bot.x
                        && (*e_next).curr.y == (*horz_edge).bot.y
                        && (*e_next).wind_delta != 0
                        && (*e_next).out_idx >= 0
                        && (*e_next).curr.y > (*e_next).top.y
                        && slopes_equal_4_points(
                            (*horz_edge).bot,
                            (*horz_edge).top,
                            (*e_next).bot,
                            (*e_next).top,
                            self.base.use_full_range,
                        )
                    {
                        let op2 = self.add_out_pt(e_next, (*horz_edge).bot);
                        self.add_join(op1, op2, (*horz_edge).top);
                    }
                } else {
                    self.base.update_edge_into_ael(&mut horz_edge)?;
                }
            } else {
                if (*horz_edge).out_idx >= 0 {
                    self.add_out_pt(horz_edge, (*horz_edge).top);
                }
                self.base.delete_from_ael(horz_edge);
            }
        }
        Ok(())
    }

    // C++: Clipper::DoMaxima
    pub unsafe fn do_maxima(&mut self, e: *mut TEdge) -> Result<()> {
        unsafe {
            let e_max_pair = get_maxima_pair_ex(e);
            if e_max_pair.is_null() {
                if (*e).out_idx >= 0 {
                    self.add_out_pt(e, (*e).top);
                }
                self.base.delete_from_ael(e);
                return Ok(());
            }

            let mut e_next = (*e).next_in_ael;
            while !e_next.is_null() && e_next != e_max_pair {
                self.intersect_edges(e, e_next, (*e).top);
                self.base.swap_positions_in_ael(e, e_next);
                e_next = (*e).next_in_ael;
            }

            if (*e).out_idx == UNASSIGNED && (*e_max_pair).out_idx == UNASSIGNED {
                self.base.delete_from_ael(e);
                self.base.delete_from_ael(e_max_pair);
            } else if (*e).out_idx >= 0 && (*e_max_pair).out_idx >= 0 {
                self.add_local_max_poly(e, e_max_pair, (*e).top);
                self.base.delete_from_ael(e);
                self.base.delete_from_ael(e_max_pair);
            } else if (*e).wind_delta == 0 {
                if (*e).out_idx >= 0 {
                    self.add_out_pt(e, (*e).top);
                    (*e).out_idx = UNASSIGNED;
                }
                self.base.delete_from_ael(e);
                if (*e_max_pair).out_idx >= 0 {
                    self.add_out_pt(e_max_pair, (*e).top);
                    (*e_max_pair).out_idx = UNASSIGNED;
                }
                self.base.delete_from_ael(e_max_pair);
            } else {
                return Err(ClipperError::new("DoMaxima error"));
            }
        }
        Ok(())
    }

    // C++: Clipper::ProcessEdgesAtTopOfScanbeam
    pub unsafe fn process_edges_at_top_of_scanbeam(&mut self, top_y: CInt) -> Result<()> {
        unsafe {
            let mut e = self.base.active_edges;
            while !e.is_null() {
                let e_top = (*e).top;
                let e_next_in_lml = (*e).next_in_lml;
                let mut is_maxima_edge = e_top.y == top_y && e_next_in_lml.is_null();

                if is_maxima_edge {
                    let e_max_pair = get_maxima_pair_ex(e);
                    is_maxima_edge = e_max_pair.is_null() || !is_horizontal(&*e_max_pair);
                }

                if is_maxima_edge {
                    if self.strict_simple {
                        self.maxima.push((*e).top.x);
                    }
                    let e_prev = (*e).prev_in_ael;
                    self.do_maxima(e)?;
                    if e_prev.is_null() {
                        e = self.base.active_edges;
                    } else {
                        e = (*e_prev).next_in_ael;
                    }
                } else {
                    let is_intermediate_edge = e_top.y == top_y && !e_next_in_lml.is_null();
                    if is_intermediate_edge && is_horizontal(&*e_next_in_lml) {
                        self.base.update_edge_into_ael(&mut e)?;
                        if (*e).out_idx >= 0 {
                            self.add_out_pt(e, (*e).bot);
                        }
                        self.add_edge_to_sel(e);
                    } else {
                        (*e).curr.x = top_x(&*e, top_y);
                        (*e).curr.y = top_y;
                    }

                    if self.strict_simple {
                        let e_prev = (*e).prev_in_ael;
                        if (*e).out_idx >= 0
                            && (*e).wind_delta != 0
                            && !e_prev.is_null()
                            && (*e_prev).out_idx >= 0
                            && (*e_prev).curr.x == (*e).curr.x
                            && (*e_prev).wind_delta != 0
                        {
                            let pt = (*e).curr;
                            let op = self.add_out_pt(e_prev, pt);
                            let op2 = self.add_out_pt(e, pt);
                            self.add_join(op, op2, pt);
                        }
                    }

                    e = (*e).next_in_ael;
                }
            }

            self.maxima.sort();
            self.process_horizontals()?;
            self.maxima.clear();

            e = self.base.active_edges;
            while !e.is_null() {
                let e_next_in_lml = (*e).next_in_lml;
                if (*e).top.y == top_y && !e_next_in_lml.is_null() {
                    let mut op: *mut OutPt = ptr::null_mut();
                    if (*e).out_idx >= 0 {
                        op = self.add_out_pt(e, (*e).top);
                    }
                    self.base.update_edge_into_ael(&mut e)?;

                    let e_prev = (*e).prev_in_ael;
                    let e_next = (*e).next_in_ael;
                    if !e_prev.is_null()
                        && (*e_prev).curr.x == (*e).bot.x
                        && (*e_prev).curr.y == (*e).bot.y
                        && !op.is_null()
                        && (*e_prev).out_idx >= 0
                        && (*e_prev).curr.y > (*e_prev).top.y
                        && slopes_equal_4_points(
                            (*e).curr,
                            (*e).top,
                            (*e_prev).curr,
                            (*e_prev).top,
                            self.base.use_full_range,
                        )
                        && (*e).wind_delta != 0
                        && (*e_prev).wind_delta != 0
                    {
                        let op2 = self.add_out_pt(e_prev, (*e).bot);
                        self.add_join(op, op2, (*e).top);
                    } else if !e_next.is_null()
                        && (*e_next).curr.x == (*e).bot.x
                        && (*e_next).curr.y == (*e).bot.y
                        && !op.is_null()
                        && (*e_next).out_idx >= 0
                        && (*e_next).curr.y > (*e_next).top.y
                        && slopes_equal_4_points(
                            (*e).curr,
                            (*e).top,
                            (*e_next).curr,
                            (*e_next).top,
                            self.base.use_full_range,
                        )
                        && (*e).wind_delta != 0
                        && (*e_next).wind_delta != 0
                    {
                        let op2 = self.add_out_pt(e_next, (*e).bot);
                        self.add_join(op, op2, (*e).top);
                    }
                }
                e = (*e).next_in_ael;
            }
        }
        Ok(())
    }

    // C++: Clipper::ProcessIntersectList
    pub unsafe fn process_intersect_list(&mut self) {
        let cnt = self.intersect_list.len();
        for i in 0..cnt {
            let node = unsafe { *self.intersect_list.get_unchecked(i) };
            unsafe {
                self.intersect_edges(node.edge1, node.edge2, node.pt);
                self.base.swap_positions_in_ael(node.edge1, node.edge2);
            }
        }
        self.intersect_list.clear();
    }

    // C++: Clipper::FixupIntersectionOrder
    pub unsafe fn fixup_intersection_order(&mut self) -> bool {
        unsafe {
            self.copy_ael_to_sel();
            self.intersect_list.sort_unstable_by(|node1, node2| {
                let y1 = node1.pt.y;
                let y2 = node2.pt.y;
                y2.cmp(&y1)
            });

            let cnt = self.intersect_list.len();
            for i in 0..cnt {
                if !edges_adjacent(self.intersect_list.get_unchecked(i)) {
                    let mut j = i + 1;
                    while j < cnt && !edges_adjacent(self.intersect_list.get_unchecked(j)) {
                        j += 1;
                    }
                    if j == cnt {
                        return false;
                    }
                    self.intersect_list.swap(i, j);
                }
                let inode = self.intersect_list.get_unchecked(i);
                self.swap_positions_in_sel(inode.edge1, inode.edge2);
            }
        }
        true
    }

    // C++: Clipper::IntersectEdges
    pub unsafe fn intersect_edges(&mut self, e1: *mut TEdge, e2: *mut TEdge, pt: IntPoint) {
        unsafe {
            let e1_contributing = (*e1).out_idx >= 0;
            let e2_contributing = (*e2).out_idx >= 0;

            if (*e1).wind_delta == 0 || (*e2).wind_delta == 0 {
                if (*e1).wind_delta == 0 && (*e2).wind_delta == 0 {
                    return;
                } else if (*e1).poly_typ == (*e2).poly_typ
                    && (*e1).wind_delta != (*e2).wind_delta
                    && self.clip_type == ClipType::Union
                {
                    if (*e1).wind_delta == 0 {
                        if e2_contributing {
                            self.add_out_pt(e1, pt);
                            if e1_contributing {
                                (*e1).out_idx = UNASSIGNED;
                            }
                        }
                    } else if e1_contributing {
                        self.add_out_pt(e2, pt);
                        if e2_contributing {
                            (*e2).out_idx = UNASSIGNED;
                        }
                    }
                } else if (*e1).poly_typ != (*e2).poly_typ {
                    if (*e1).wind_delta == 0
                        && abs((*e2).wind_cnt as CInt) == 1
                        && (self.clip_type != ClipType::Union || (*e2).wind_cnt2 == 0)
                    {
                        self.add_out_pt(e1, pt);
                        if e1_contributing {
                            (*e1).out_idx = UNASSIGNED;
                        }
                    } else if (*e2).wind_delta == 0
                        && abs((*e1).wind_cnt as CInt) == 1
                        && (self.clip_type != ClipType::Union || (*e1).wind_cnt2 == 0)
                    {
                        self.add_out_pt(e2, pt);
                        if e2_contributing {
                            (*e2).out_idx = UNASSIGNED;
                        }
                    }
                }
                return;
            }

            if (*e1).poly_typ == (*e2).poly_typ {
                if self.is_even_odd_fill_type(&*e1) {
                    let old_e1_wind_cnt = (*e1).wind_cnt;
                    (*e1).wind_cnt = (*e2).wind_cnt;
                    (*e2).wind_cnt = old_e1_wind_cnt;
                } else {
                    if (*e1).wind_cnt + (*e2).wind_delta == 0 {
                        (*e1).wind_cnt = -(*e1).wind_cnt;
                    } else {
                        (*e1).wind_cnt += (*e2).wind_delta;
                    }
                    if (*e2).wind_cnt - (*e1).wind_delta == 0 {
                        (*e2).wind_cnt = -(*e2).wind_cnt;
                    } else {
                        (*e2).wind_cnt -= (*e1).wind_delta;
                    }
                }
            } else {
                if !self.is_even_odd_fill_type(&*e2) {
                    (*e1).wind_cnt2 += (*e2).wind_delta;
                } else {
                    (*e1).wind_cnt2 = if (*e1).wind_cnt2 == 0 { 1 } else { 0 };
                }
                if !self.is_even_odd_fill_type(&*e1) {
                    (*e2).wind_cnt2 -= (*e1).wind_delta;
                } else {
                    (*e2).wind_cnt2 = if (*e2).wind_cnt2 == 0 { 1 } else { 0 };
                }
            }

            let (e1_fill_type, e1_fill_type2) = if (*e1).poly_typ == PolyType::Subject {
                (self.subj_fill_type, self.clip_fill_type)
            } else {
                (self.clip_fill_type, self.subj_fill_type)
            };
            let (e2_fill_type, e2_fill_type2) = if (*e2).poly_typ == PolyType::Subject {
                (self.subj_fill_type, self.clip_fill_type)
            } else {
                (self.clip_fill_type, self.subj_fill_type)
            };

            let e1_wc = winding_count_for_fill(e1_fill_type, (*e1).wind_cnt);
            let e2_wc = winding_count_for_fill(e2_fill_type, (*e2).wind_cnt);

            if e1_contributing && e2_contributing {
                if (e1_wc != 0 && e1_wc != 1)
                    || (e2_wc != 0 && e2_wc != 1)
                    || ((*e1).poly_typ != (*e2).poly_typ && self.clip_type != ClipType::Xor)
                {
                    self.add_local_max_poly(e1, e2, pt);
                } else {
                    self.add_out_pt(e1, pt);
                    self.add_out_pt(e2, pt);
                    swap_sides(&mut *e1, &mut *e2);
                    swap_poly_indexes(&mut *e1, &mut *e2);
                }
            } else if e1_contributing {
                if e2_wc == 0 || e2_wc == 1 {
                    self.add_out_pt(e1, pt);
                    swap_sides(&mut *e1, &mut *e2);
                    swap_poly_indexes(&mut *e1, &mut *e2);
                }
            } else if e2_contributing {
                if e1_wc == 0 || e1_wc == 1 {
                    self.add_out_pt(e2, pt);
                    swap_sides(&mut *e1, &mut *e2);
                    swap_poly_indexes(&mut *e1, &mut *e2);
                }
            } else if (e1_wc == 0 || e1_wc == 1) && (e2_wc == 0 || e2_wc == 1) {
                let e1_wc2 = winding_count_for_fill(e1_fill_type2, (*e1).wind_cnt2);
                let e2_wc2 = winding_count_for_fill(e2_fill_type2, (*e2).wind_cnt2);

                if (*e1).poly_typ != (*e2).poly_typ {
                    self.add_local_min_poly(e1, e2, pt);
                } else if e1_wc == 1 && e2_wc == 1 {
                    match self.clip_type {
                        ClipType::Intersection => {
                            if e1_wc2 > 0 && e2_wc2 > 0 {
                                self.add_local_min_poly(e1, e2, pt);
                            }
                        }
                        ClipType::Union => {
                            if e1_wc2 <= 0 && e2_wc2 <= 0 {
                                self.add_local_min_poly(e1, e2, pt);
                            }
                        }
                        ClipType::Difference => {
                            if ((*e1).poly_typ == PolyType::Clip && e1_wc2 > 0 && e2_wc2 > 0)
                                || ((*e1).poly_typ == PolyType::Subject
                                    && e1_wc2 <= 0
                                    && e2_wc2 <= 0)
                            {
                                self.add_local_min_poly(e1, e2, pt);
                            }
                        }
                        ClipType::Xor => {
                            self.add_local_min_poly(e1, e2, pt);
                        }
                    }
                } else {
                    swap_sides(&mut *e1, &mut *e2);
                }
            }
        }
    }

    // C++: Clipper::SetWindingCount
    pub unsafe fn set_winding_count(&mut self, edge: *mut TEdge) {
        unsafe {
            let mut e = (*edge).prev_in_ael;
            while !e.is_null() && ((*e).poly_typ != (*edge).poly_typ || (*e).wind_delta == 0) {
                e = (*e).prev_in_ael;
            }

            if e.is_null() {
                if (*edge).wind_delta == 0 {
                    let pft = if (*edge).poly_typ == PolyType::Subject {
                        self.subj_fill_type
                    } else {
                        self.clip_fill_type
                    };
                    (*edge).wind_cnt = if pft == PolyFillType::Negative { -1 } else { 1 };
                } else {
                    (*edge).wind_cnt = (*edge).wind_delta;
                }
                (*edge).wind_cnt2 = 0;
                e = self.base.active_edges;
            } else if (*edge).wind_delta == 0 && self.clip_type != ClipType::Union {
                (*edge).wind_cnt = 1;
                (*edge).wind_cnt2 = (*e).wind_cnt2;
                e = (*e).next_in_ael;
            } else if self.is_even_odd_fill_type(&*edge) {
                if (*edge).wind_delta == 0 {
                    let mut inside = true;
                    let mut e2 = (*e).prev_in_ael;
                    while !e2.is_null() {
                        if (*e2).poly_typ == (*e).poly_typ && (*e2).wind_delta != 0 {
                            inside = !inside;
                        }
                        e2 = (*e2).prev_in_ael;
                    }
                    (*edge).wind_cnt = if inside { 0 } else { 1 };
                } else {
                    (*edge).wind_cnt = (*edge).wind_delta;
                }
                (*edge).wind_cnt2 = (*e).wind_cnt2;
                e = (*e).next_in_ael;
            } else {
                if (*e).wind_cnt * (*e).wind_delta < 0 {
                    if (*e).wind_cnt.abs() > 1 {
                        if (*e).wind_delta * (*edge).wind_delta < 0 {
                            (*edge).wind_cnt = (*e).wind_cnt;
                        } else {
                            (*edge).wind_cnt = (*e).wind_cnt + (*edge).wind_delta;
                        }
                    } else {
                        (*edge).wind_cnt = if (*edge).wind_delta == 0 {
                            1
                        } else {
                            (*edge).wind_delta
                        };
                    }
                } else if (*edge).wind_delta == 0 {
                    (*edge).wind_cnt = if (*e).wind_cnt < 0 {
                        (*e).wind_cnt - 1
                    } else {
                        (*e).wind_cnt + 1
                    };
                } else if (*e).wind_delta * (*edge).wind_delta < 0 {
                    (*edge).wind_cnt = (*e).wind_cnt;
                } else {
                    (*edge).wind_cnt = (*e).wind_cnt + (*edge).wind_delta;
                }
                (*edge).wind_cnt2 = (*e).wind_cnt2;
                e = (*e).next_in_ael;
            }

            if self.is_even_odd_alt_fill_type(&*edge) {
                while e != edge {
                    if (*e).wind_delta != 0 {
                        (*edge).wind_cnt2 = if (*edge).wind_cnt2 == 0 { 1 } else { 0 };
                    }
                    e = (*e).next_in_ael;
                }
            } else {
                while e != edge {
                    (*edge).wind_cnt2 += (*e).wind_delta;
                    e = (*e).next_in_ael;
                }
            }
        }
    }

    // C++: Clipper::IsEvenOddFillType
    pub fn is_even_odd_fill_type(&self, edge: &TEdge) -> bool {
        if edge.poly_typ == PolyType::Subject {
            self.subj_fill_type == PolyFillType::EvenOdd
        } else {
            self.clip_fill_type == PolyFillType::EvenOdd
        }
    }

    // C++: Clipper::IsEvenOddAltFillType
    pub fn is_even_odd_alt_fill_type(&self, edge: &TEdge) -> bool {
        if edge.poly_typ == PolyType::Subject {
            self.clip_fill_type == PolyFillType::EvenOdd
        } else {
            self.subj_fill_type == PolyFillType::EvenOdd
        }
    }

    // C++: Clipper::IsContributing
    pub fn is_contributing(&self, edge: &TEdge) -> bool {
        let (pft, pft2) = if edge.poly_typ == PolyType::Subject {
            (self.subj_fill_type, self.clip_fill_type)
        } else {
            (self.clip_fill_type, self.subj_fill_type)
        };

        match pft {
            PolyFillType::EvenOdd => {
                if edge.wind_delta == 0 && edge.wind_cnt != 1 {
                    return false;
                }
            }
            PolyFillType::NonZero => {
                if edge.wind_cnt.abs() != 1 {
                    return false;
                }
            }
            PolyFillType::Positive => {
                if edge.wind_cnt != 1 {
                    return false;
                }
            }
            PolyFillType::Negative => {
                if edge.wind_cnt != -1 {
                    return false;
                }
            }
        }

        match self.clip_type {
            ClipType::Intersection => match pft2 {
                PolyFillType::EvenOdd | PolyFillType::NonZero => edge.wind_cnt2 != 0,
                PolyFillType::Positive => edge.wind_cnt2 > 0,
                PolyFillType::Negative => edge.wind_cnt2 < 0,
            },
            ClipType::Union => match pft2 {
                PolyFillType::EvenOdd | PolyFillType::NonZero => edge.wind_cnt2 == 0,
                PolyFillType::Positive => edge.wind_cnt2 <= 0,
                PolyFillType::Negative => edge.wind_cnt2 >= 0,
            },
            ClipType::Difference => {
                if edge.poly_typ == PolyType::Subject {
                    match pft2 {
                        PolyFillType::EvenOdd | PolyFillType::NonZero => edge.wind_cnt2 == 0,
                        PolyFillType::Positive => edge.wind_cnt2 <= 0,
                        PolyFillType::Negative => edge.wind_cnt2 >= 0,
                    }
                } else {
                    match pft2 {
                        PolyFillType::EvenOdd | PolyFillType::NonZero => edge.wind_cnt2 != 0,
                        PolyFillType::Positive => edge.wind_cnt2 > 0,
                        PolyFillType::Negative => edge.wind_cnt2 < 0,
                    }
                }
            }
            ClipType::Xor => {
                if edge.wind_delta == 0 {
                    match pft2 {
                        PolyFillType::EvenOdd | PolyFillType::NonZero => edge.wind_cnt2 == 0,
                        PolyFillType::Positive => edge.wind_cnt2 <= 0,
                        PolyFillType::Negative => edge.wind_cnt2 >= 0,
                    }
                } else {
                    true
                }
            }
        }
    }

    // C++: Clipper::AddJoin
    pub fn add_join(
        &mut self,
        op1: *mut crate::types::OutPt,
        op2: *mut crate::types::OutPt,
        off_pt: IntPoint,
    ) {
        let join = Join {
            out_pt1: op1,
            out_pt2: op2,
            off_pt,
        };
        self.joins.push(join);
    }

    // C++: Clipper::ClearJoins
    pub unsafe fn clear_joins(&mut self) {
        self.joins.clear();
    }

    // C++: Clipper::AddGhostJoin
    pub fn add_ghost_join(&mut self, op: *mut crate::types::OutPt, off_pt: IntPoint) {
        let join = Join {
            out_pt1: op,
            out_pt2: ptr::null_mut(),
            off_pt,
        };
        self.ghost_joins.push(join);
    }

    // C++: Clipper::ClearGhostJoins
    pub unsafe fn clear_ghost_joins(&mut self) {
        self.ghost_joins.clear();
    }

    // C++: Clipper::DisposeIntersectNodes
    pub unsafe fn dispose_intersect_nodes(&mut self) {
        self.intersect_list.clear();
    }

    // C++: Clipper::FixupOutPolyline
    pub unsafe fn fixup_out_polyline(&mut self, outrec: *mut OutRec) {
        unsafe {
            let mut pp = (*outrec).pts;
            let mut last_pp = (*pp).prev;
            while pp != last_pp {
                pp = (*pp).next;
                if (*pp).pt == (*(*pp).prev).pt {
                    if pp == last_pp {
                        last_pp = (*pp).prev;
                    }
                    let tmp_pp = (*pp).prev;
                    (*tmp_pp).next = (*pp).next;
                    (*(*pp).next).prev = tmp_pp;
                    pp = tmp_pp;
                }
            }

            if pp == (*pp).prev {
                (*outrec).pts = ptr::null_mut();
            }
        }
    }

    // C++: Clipper::FixupOutPolygon
    pub unsafe fn fixup_out_polygon(&mut self, outrec: *mut OutRec) {
        unsafe {
            let mut last_ok: *mut OutPt = ptr::null_mut();
            (*outrec).bottom_pt = ptr::null_mut();
            let mut pp = (*outrec).pts;
            let preserve_col = self.base.preserve_collinear || self.strict_simple;

            loop {
                if (*pp).prev == pp || (*pp).prev == (*pp).next {
                    (*outrec).pts = ptr::null_mut();
                    return;
                }

                if (*pp).pt == (*(*pp).next).pt
                    || (*pp).pt == (*(*pp).prev).pt
                    || (slopes_equal_3_points(
                        (*(*pp).prev).pt,
                        (*pp).pt,
                        (*(*pp).next).pt,
                        self.base.use_full_range,
                    ) && (!preserve_col
                        || !pt2_is_between_pt1_and_pt3(
                            (*(*pp).prev).pt,
                            (*pp).pt,
                            (*(*pp).next).pt,
                        )))
                {
                    last_ok = ptr::null_mut();
                    (*(*pp).prev).next = (*pp).next;
                    (*(*pp).next).prev = (*pp).prev;
                    pp = (*pp).prev;
                } else if pp == last_ok {
                    break;
                } else {
                    if last_ok.is_null() {
                        last_ok = pp;
                    }
                    pp = (*pp).next;
                }
            }
            (*outrec).pts = pp;
        }
    }

    // C++: Clipper::BuildResult
    pub unsafe fn build_result(&self, polys: &mut Paths) {
        unsafe {
            polys.reserve(self.base.poly_outs.len());
            for outrec in &self.base.poly_outs {
                if (*outrec).is_null() || (*(*outrec)).pts.is_null() {
                    continue;
                }
                let mut p = (*(*(*outrec)).pts).prev;
                let cnt = point_count(p);
                if cnt < 2 {
                    continue;
                }
                let mut pg = Vec::with_capacity(cnt);
                for _ in 0..cnt {
                    pg.push((*p).pt);
                    p = (*p).prev;
                }
                polys.push(pg);
            }
        }
    }

    // C++: Clipper::BuildResult2
    pub unsafe fn build_result2(&mut self, polytree: &mut PolyTree) {
        unsafe {
            polytree.clear();
            polytree.all_nodes.reserve(self.base.poly_outs.len());

            for i in 0..self.base.poly_outs.len() {
                let outrec = self.base.poly_outs[i];
                let cnt = point_count((*outrec).pts);
                if ((*outrec).is_open && cnt < 2) || (!(*outrec).is_open && cnt < 3) {
                    continue;
                }
                self.fix_hole_linkage(outrec);
                let pn = Box::into_raw(Box::new(PolyNode::new()));
                polytree.all_nodes.push(pn);
                (*outrec).poly_nd = pn;
                (*pn).parent = ptr::null_mut();
                (*pn).index = 0;
                (*pn).contour.reserve(cnt);
                let mut op = (*(*outrec).pts).prev;
                for _ in 0..cnt {
                    (*pn).contour.push((*op).pt);
                    op = (*op).prev;
                }
            }

            polytree.node.childs.reserve(self.base.poly_outs.len());
            for i in 0..self.base.poly_outs.len() {
                let outrec = self.base.poly_outs[i];
                if (*outrec).poly_nd.is_null() {
                    continue;
                }
                if (*outrec).is_open {
                    (*(*outrec).poly_nd).is_open = true;
                    polytree.node.add_child((*outrec).poly_nd);
                } else if !(*outrec).first_left.is_null()
                    && !(*(*outrec).first_left).poly_nd.is_null()
                {
                    (*(*(*outrec).first_left).poly_nd).add_child((*outrec).poly_nd);
                } else {
                    polytree.node.add_child((*outrec).poly_nd);
                }
            }
        }
    }

    // C++: Clipper::FixupFirstLefts1
    pub unsafe fn fixup_first_lefts1(
        &mut self,
        old_out_rec: *mut OutRec,
        new_out_rec: *mut OutRec,
    ) {
        unsafe {
            for outrec in &self.base.poly_outs {
                let first_left = parse_first_left((**outrec).first_left);
                if !(**outrec).pts.is_null()
                    && first_left == old_out_rec
                    && poly2_contains_poly1((**outrec).pts, (*new_out_rec).pts)
                {
                    (**outrec).first_left = new_out_rec;
                }
            }
        }
    }

    // C++: Clipper::FixupFirstLefts2
    pub unsafe fn fixup_first_lefts2(
        &mut self,
        inner_out_rec: *mut OutRec,
        outer_out_rec: *mut OutRec,
    ) {
        unsafe {
            let orfl = (*outer_out_rec).first_left;
            for outrec in &self.base.poly_outs {
                if (**outrec).pts.is_null() || *outrec == outer_out_rec || *outrec == inner_out_rec
                {
                    continue;
                }
                let first_left = parse_first_left((**outrec).first_left);
                if first_left != orfl && first_left != inner_out_rec && first_left != outer_out_rec
                {
                    continue;
                }
                if poly2_contains_poly1((**outrec).pts, (*inner_out_rec).pts) {
                    (**outrec).first_left = inner_out_rec;
                } else if poly2_contains_poly1((**outrec).pts, (*outer_out_rec).pts) {
                    (**outrec).first_left = outer_out_rec;
                } else if (**outrec).first_left == inner_out_rec
                    || (**outrec).first_left == outer_out_rec
                {
                    (**outrec).first_left = orfl;
                }
            }
        }
    }

    // C++: Clipper::FixupFirstLefts3
    pub unsafe fn fixup_first_lefts3(
        &mut self,
        old_out_rec: *mut OutRec,
        new_out_rec: *mut OutRec,
    ) {
        unsafe {
            for outrec in &self.base.poly_outs {
                let first_left = parse_first_left((**outrec).first_left);
                if !(**outrec).pts.is_null() && first_left == old_out_rec {
                    (**outrec).first_left = new_out_rec;
                }
            }
        }
    }

    // C++: Clipper::JoinPoints
    pub unsafe fn join_points(
        &mut self,
        j: &mut Join,
        out_rec1: *mut OutRec,
        out_rec2: *mut OutRec,
    ) -> bool {
        unsafe {
            let op1 = j.out_pt1;
            let mut op1b: *mut OutPt;
            let op2 = j.out_pt2;
            let mut op2b: *mut OutPt;

            let is_horizontal = (*j.out_pt1).pt.y == j.off_pt.y;

            if is_horizontal && j.off_pt == (*j.out_pt1).pt && j.off_pt == (*j.out_pt2).pt {
                if out_rec1 != out_rec2 {
                    return false;
                }
                op1b = (*j.out_pt1).next;
                while op1b != op1 && (*op1b).pt == j.off_pt {
                    op1b = (*op1b).next;
                }
                let reverse1 = (*op1b).pt.y > j.off_pt.y;
                op2b = (*j.out_pt2).next;
                while op2b != op2 && (*op2b).pt == j.off_pt {
                    op2b = (*op2b).next;
                }
                let reverse2 = (*op2b).pt.y > j.off_pt.y;
                if reverse1 == reverse2 {
                    return false;
                }
                if reverse1 {
                    op1b = dup_out_pt_arena(&mut self.base, op1, false);
                    op2b = dup_out_pt_arena(&mut self.base, op2, true);
                    (*op1).prev = op2;
                    (*op2).next = op1;
                    (*op1b).next = op2b;
                    (*op2b).prev = op1b;
                    j.out_pt1 = op1;
                    j.out_pt2 = op1b;
                    true
                } else {
                    op1b = dup_out_pt_arena(&mut self.base, op1, true);
                    op2b = dup_out_pt_arena(&mut self.base, op2, false);
                    (*op1).next = op2;
                    (*op2).prev = op1;
                    (*op1b).prev = op2b;
                    (*op2b).next = op1b;
                    j.out_pt1 = op1;
                    j.out_pt2 = op1b;
                    true
                }
            } else if is_horizontal {
                let mut op1 = op1;
                op1b = op1;
                while (*(*op1).prev).pt.y == (*op1).pt.y
                    && (*op1).prev != op1b
                    && (*op1).prev != op2
                {
                    op1 = (*op1).prev;
                }
                while (*(*op1b).next).pt.y == (*op1b).pt.y
                    && (*op1b).next != op1
                    && (*op1b).next != op2
                {
                    op1b = (*op1b).next;
                }
                if (*op1b).next == op1 || (*op1b).next == op2 {
                    return false;
                }

                let mut op2 = op2;
                op2b = op2;
                while (*(*op2).prev).pt.y == (*op2).pt.y
                    && (*op2).prev != op2b
                    && (*op2).prev != op1b
                {
                    op2 = (*op2).prev;
                }
                while (*(*op2b).next).pt.y == (*op2b).pt.y
                    && (*op2b).next != op2
                    && (*op2b).next != op1
                {
                    op2b = (*op2b).next;
                }
                if (*op2b).next == op2 || (*op2b).next == op1 {
                    return false;
                }

                let Some((left, right)) =
                    get_overlap((*op1).pt.x, (*op1b).pt.x, (*op2).pt.x, (*op2b).pt.x)
                else {
                    return false;
                };

                let (pt, discard_left_side) = if (*op1).pt.x >= left && (*op1).pt.x <= right {
                    ((*op1).pt, (*op1).pt.x > (*op1b).pt.x)
                } else if (*op2).pt.x >= left && (*op2).pt.x <= right {
                    ((*op2).pt, (*op2).pt.x > (*op2b).pt.x)
                } else if (*op1b).pt.x >= left && (*op1b).pt.x <= right {
                    ((*op1b).pt, (*op1b).pt.x > (*op1).pt.x)
                } else {
                    ((*op2b).pt, (*op2b).pt.x > (*op2).pt.x)
                };
                j.out_pt1 = op1;
                j.out_pt2 = op2;
                join_horz(&mut self.base, op1, op1b, op2, op2b, pt, discard_left_side)
            } else {
                op1b = (*op1).next;
                while (*op1b).pt == (*op1).pt && op1b != op1 {
                    op1b = (*op1b).next;
                }
                let reverse1 = (*op1b).pt.y > (*op1).pt.y
                    || !slopes_equal_3_points(
                        (*op1).pt,
                        (*op1b).pt,
                        j.off_pt,
                        self.base.use_full_range,
                    );
                if reverse1 {
                    op1b = (*op1).prev;
                    while (*op1b).pt == (*op1).pt && op1b != op1 {
                        op1b = (*op1b).prev;
                    }
                    if (*op1b).pt.y > (*op1).pt.y
                        || !slopes_equal_3_points(
                            (*op1).pt,
                            (*op1b).pt,
                            j.off_pt,
                            self.base.use_full_range,
                        )
                    {
                        return false;
                    }
                }

                op2b = (*op2).next;
                while (*op2b).pt == (*op2).pt && op2b != op2 {
                    op2b = (*op2b).next;
                }
                let reverse2 = (*op2b).pt.y > (*op2).pt.y
                    || !slopes_equal_3_points(
                        (*op2).pt,
                        (*op2b).pt,
                        j.off_pt,
                        self.base.use_full_range,
                    );
                if reverse2 {
                    op2b = (*op2).prev;
                    while (*op2b).pt == (*op2).pt && op2b != op2 {
                        op2b = (*op2b).prev;
                    }
                    if (*op2b).pt.y > (*op2).pt.y
                        || !slopes_equal_3_points(
                            (*op2).pt,
                            (*op2b).pt,
                            j.off_pt,
                            self.base.use_full_range,
                        )
                    {
                        return false;
                    }
                }

                if op1b == op1
                    || op2b == op2
                    || op1b == op2b
                    || (out_rec1 == out_rec2 && reverse1 == reverse2)
                {
                    return false;
                }

                if reverse1 {
                    op1b = dup_out_pt_arena(&mut self.base, op1, false);
                    op2b = dup_out_pt_arena(&mut self.base, op2, true);
                    (*op1).prev = op2;
                    (*op2).next = op1;
                    (*op1b).next = op2b;
                    (*op2b).prev = op1b;
                    j.out_pt1 = op1;
                    j.out_pt2 = op1b;
                    true
                } else {
                    op1b = dup_out_pt_arena(&mut self.base, op1, true);
                    op2b = dup_out_pt_arena(&mut self.base, op2, false);
                    (*op1).next = op2;
                    (*op2).prev = op1;
                    (*op1b).prev = op2b;
                    (*op2b).next = op1b;
                    j.out_pt1 = op1;
                    j.out_pt2 = op1b;
                    true
                }
            }
        }
    }

    // C++: Clipper::JoinCommonEdges
    pub unsafe fn join_common_edges(&mut self) {
        unsafe {
            let mut joins = std::mem::take(&mut self.joins);
            for join in &mut joins {
                let out_rec1 = self.get_out_rec((*join.out_pt1).idx);
                let mut out_rec2 = self.get_out_rec((*join.out_pt2).idx);

                if (*out_rec1).pts.is_null() || (*out_rec2).pts.is_null() {
                    continue;
                }
                if (*out_rec1).is_open || (*out_rec2).is_open {
                    continue;
                }

                let hole_state_rec = if out_rec1 == out_rec2 {
                    out_rec1
                } else if out_rec1_right_of_out_rec2(out_rec1, out_rec2) {
                    out_rec2
                } else if out_rec1_right_of_out_rec2(out_rec2, out_rec1) {
                    out_rec1
                } else {
                    get_lowermost_rec(out_rec1, out_rec2)
                };

                if !self.join_points(join, out_rec1, out_rec2) {
                    continue;
                }

                if out_rec1 == out_rec2 {
                    (*out_rec1).pts = join.out_pt1;
                    (*out_rec1).bottom_pt = ptr::null_mut();
                    out_rec2 = self.base.create_out_rec();
                    (*out_rec2).pts = join.out_pt2;

                    update_out_pt_idxs(out_rec2);

                    if poly2_contains_poly1((*out_rec2).pts, (*out_rec1).pts) {
                        (*out_rec2).is_hole = !(*out_rec1).is_hole;
                        (*out_rec2).first_left = out_rec1;

                        if self.using_poly_tree {
                            self.fixup_first_lefts2(out_rec2, out_rec1);
                        }

                        if ((*out_rec2).is_hole ^ self.reverse_output)
                            == (area_out_pt((*out_rec2).pts) > 0.0)
                        {
                            reverse_poly_pt_links((*out_rec2).pts);
                        }
                    } else if poly2_contains_poly1((*out_rec1).pts, (*out_rec2).pts) {
                        (*out_rec2).is_hole = (*out_rec1).is_hole;
                        (*out_rec1).is_hole = !(*out_rec2).is_hole;
                        (*out_rec2).first_left = (*out_rec1).first_left;
                        (*out_rec1).first_left = out_rec2;

                        if self.using_poly_tree {
                            self.fixup_first_lefts2(out_rec1, out_rec2);
                        }

                        if ((*out_rec1).is_hole ^ self.reverse_output)
                            == (area_out_pt((*out_rec1).pts) > 0.0)
                        {
                            reverse_poly_pt_links((*out_rec1).pts);
                        }
                    } else {
                        (*out_rec2).is_hole = (*out_rec1).is_hole;
                        (*out_rec2).first_left = (*out_rec1).first_left;

                        if self.using_poly_tree {
                            self.fixup_first_lefts1(out_rec1, out_rec2);
                        }
                    }
                } else {
                    (*out_rec2).pts = ptr::null_mut();
                    (*out_rec2).bottom_pt = ptr::null_mut();
                    (*out_rec2).idx = (*out_rec1).idx;

                    (*out_rec1).is_hole = (*hole_state_rec).is_hole;
                    if hole_state_rec == out_rec2 {
                        (*out_rec1).first_left = (*out_rec2).first_left;
                    }
                    (*out_rec2).first_left = out_rec1;

                    if self.using_poly_tree {
                        self.fixup_first_lefts3(out_rec2, out_rec1);
                    }
                }
            }
            self.joins = joins;
        }
    }

    // C++: Clipper::DoSimplePolygons
    pub unsafe fn do_simple_polygons(&mut self) {
        unsafe {
            let mut i = 0;
            while i < self.base.poly_outs.len() {
                let outrec = self.base.poly_outs[i];
                i += 1;
                let mut op = (*outrec).pts;
                if op.is_null() || (*outrec).is_open {
                    continue;
                }
                loop {
                    let mut op2 = (*op).next;
                    while op2 != (*outrec).pts {
                        if (*op).pt == (*op2).pt && (*op2).next != op && (*op2).prev != op {
                            let op3 = (*op).prev;
                            let op4 = (*op2).prev;
                            (*op).prev = op4;
                            (*op4).next = op;
                            (*op2).prev = op3;
                            (*op3).next = op2;

                            (*outrec).pts = op;
                            let outrec2 = self.base.create_out_rec();
                            (*outrec2).pts = op2;
                            update_out_pt_idxs(outrec2);
                            if poly2_contains_poly1((*outrec2).pts, (*outrec).pts) {
                                (*outrec2).is_hole = !(*outrec).is_hole;
                                (*outrec2).first_left = outrec;
                                if self.using_poly_tree {
                                    self.fixup_first_lefts2(outrec2, outrec);
                                }
                            } else if poly2_contains_poly1((*outrec).pts, (*outrec2).pts) {
                                (*outrec2).is_hole = (*outrec).is_hole;
                                (*outrec).is_hole = !(*outrec2).is_hole;
                                (*outrec2).first_left = (*outrec).first_left;
                                (*outrec).first_left = outrec2;
                                if self.using_poly_tree {
                                    self.fixup_first_lefts2(outrec, outrec2);
                                }
                            } else {
                                (*outrec2).is_hole = (*outrec).is_hole;
                                (*outrec2).first_left = (*outrec).first_left;
                                if self.using_poly_tree {
                                    self.fixup_first_lefts1(outrec, outrec2);
                                }
                            }
                            op2 = op;
                        }
                        op2 = (*op2).next;
                    }
                    op = (*op).next;
                    if op == (*outrec).pts {
                        break;
                    }
                }
            }
        }
    }
}

impl Drop for Clipper {
    fn drop(&mut self) {
        unsafe {
            self.clear_joins();
            self.clear_ghost_joins();
            self.dispose_intersect_nodes();
        }
    }
}

// C++: GetLowermostRec
unsafe fn get_lowermost_rec(out_rec1: *mut OutRec, out_rec2: *mut OutRec) -> *mut OutRec {
    unsafe {
        if (*out_rec1).bottom_pt.is_null() {
            (*out_rec1).bottom_pt = get_bottom_pt((*out_rec1).pts);
        }
        if (*out_rec2).bottom_pt.is_null() {
            (*out_rec2).bottom_pt = get_bottom_pt((*out_rec2).pts);
        }
        let out_pt1 = (*out_rec1).bottom_pt;
        let out_pt2 = (*out_rec2).bottom_pt;
        if (*out_pt1).pt.y > (*out_pt2).pt.y {
            out_rec1
        } else if (*out_pt1).pt.y < (*out_pt2).pt.y {
            out_rec2
        } else if (*out_pt1).pt.x < (*out_pt2).pt.x {
            out_rec1
        } else if (*out_pt1).pt.x > (*out_pt2).pt.x {
            out_rec2
        } else if (*out_pt1).next == out_pt1 {
            out_rec2
        } else if (*out_pt2).next == out_pt2 {
            out_rec1
        } else if first_is_bottom_pt(out_pt1, out_pt2) {
            out_rec1
        } else {
            out_rec2
        }
    }
}

// C++: OutRec1RightOfOutRec2
unsafe fn out_rec1_right_of_out_rec2(mut out_rec1: *mut OutRec, out_rec2: *mut OutRec) -> bool {
    unsafe {
        loop {
            out_rec1 = (*out_rec1).first_left;
            if out_rec1 == out_rec2 {
                return true;
            }
            if out_rec1.is_null() {
                return false;
            }
        }
    }
}

// C++: GetNextInAEL
#[allow(dead_code)]
unsafe fn get_next_in_ael(e: *mut TEdge, dir: Direction) -> *mut TEdge {
    unsafe {
        if dir == Direction::LeftToRight {
            (*e).next_in_ael
        } else {
            (*e).prev_in_ael
        }
    }
}

// C++: IsMinima
#[allow(dead_code)]
unsafe fn is_minima(e: *mut TEdge) -> bool {
    unsafe { !e.is_null() && (*(*e).prev).next_in_lml != e && (*(*e).next).next_in_lml != e }
}

// C++: IsMaxima
#[allow(dead_code)]
unsafe fn is_maxima(e: *mut TEdge, y: CInt) -> bool {
    unsafe { !e.is_null() && (*e).top.y == y && (*e).next_in_lml.is_null() }
}

// C++: IsIntermediate
#[allow(dead_code)]
unsafe fn is_intermediate(e: *mut TEdge, y: CInt) -> bool {
    unsafe { (*e).top.y == y && !(*e).next_in_lml.is_null() }
}

// C++: GetMaximaPair
unsafe fn get_maxima_pair(e: *mut TEdge) -> *mut TEdge {
    unsafe {
        if (*(*e).next).top == (*e).top && (*(*e).next).next_in_lml.is_null() {
            (*e).next
        } else if (*(*e).prev).top == (*e).top && (*(*e).prev).next_in_lml.is_null() {
            (*e).prev
        } else {
            ptr::null_mut()
        }
    }
}

// C++: GetMaximaPairEx
unsafe fn get_maxima_pair_ex(e: *mut TEdge) -> *mut TEdge {
    unsafe {
        let result = get_maxima_pair(e);
        if !result.is_null()
            && ((*result).out_idx == SKIP
                || ((*result).next_in_ael == (*result).prev_in_ael && !is_horizontal(&*result)))
        {
            ptr::null_mut()
        } else {
            result
        }
    }
}

// C++: PointCount
unsafe fn point_count(pts: *mut OutPt) -> usize {
    if pts.is_null() {
        return 0;
    }

    unsafe {
        let mut result = 0;
        let mut p = pts;
        loop {
            result += 1;
            p = (*p).next;
            if p == pts {
                break;
            }
        }
        result
    }
}

// C++: UpdateOutPtIdxs
#[allow(dead_code)]
unsafe fn update_out_pt_idxs(outrec: *mut OutRec) {
    unsafe {
        let mut op = (*outrec).pts;
        loop {
            (*op).idx = (*outrec).idx;
            op = (*op).prev;
            if op == (*outrec).pts {
                break;
            }
        }
    }
}

// C++: DupOutPt
#[allow(dead_code)]
unsafe fn dup_out_pt(out_pt: *mut OutPt, insert_after: bool) -> *mut OutPt {
    unsafe {
        let result = Box::into_raw(Box::new(OutPt {
            pt: (*out_pt).pt,
            idx: (*out_pt).idx,
            next: ptr::null_mut(),
            prev: ptr::null_mut(),
        }));
        if insert_after {
            (*result).next = (*out_pt).next;
            (*result).prev = out_pt;
            (*(*out_pt).next).prev = result;
            (*out_pt).next = result;
        } else {
            (*result).prev = (*out_pt).prev;
            (*result).next = out_pt;
            (*(*out_pt).prev).next = result;
            (*out_pt).prev = result;
        }
        result
    }
}

unsafe fn dup_out_pt_arena(
    base: &mut ClipperBase,
    out_pt: *mut OutPt,
    insert_after: bool,
) -> *mut OutPt {
    unsafe {
        let result = base.create_out_pt(OutPt {
            pt: (*out_pt).pt,
            idx: (*out_pt).idx,
            next: ptr::null_mut(),
            prev: ptr::null_mut(),
        });
        if insert_after {
            (*result).next = (*out_pt).next;
            (*result).prev = out_pt;
            (*(*out_pt).next).prev = result;
            (*out_pt).next = result;
        } else {
            (*result).prev = (*out_pt).prev;
            (*result).next = out_pt;
            (*(*out_pt).prev).next = result;
            (*out_pt).prev = result;
        }
        result
    }
}

// C++: JoinHorz
unsafe fn join_horz(
    base: &mut ClipperBase,
    mut op1: *mut OutPt,
    mut op1b: *mut OutPt,
    mut op2: *mut OutPt,
    mut op2b: *mut OutPt,
    pt: IntPoint,
    discard_left: bool,
) -> bool {
    unsafe {
        let dir1 = if (*op1).pt.x > (*op1b).pt.x {
            Direction::RightToLeft
        } else {
            Direction::LeftToRight
        };
        let dir2 = if (*op2).pt.x > (*op2b).pt.x {
            Direction::RightToLeft
        } else {
            Direction::LeftToRight
        };
        if dir1 == dir2 {
            return false;
        }

        if dir1 == Direction::LeftToRight {
            while (*(*op1).next).pt.x <= pt.x
                && (*(*op1).next).pt.x >= (*op1).pt.x
                && (*(*op1).next).pt.y == pt.y
            {
                op1 = (*op1).next;
            }
            if discard_left && (*op1).pt.x != pt.x {
                op1 = (*op1).next;
            }
            op1b = dup_out_pt_arena(base, op1, !discard_left);
            if (*op1b).pt != pt {
                op1 = op1b;
                (*op1).pt = pt;
                op1b = dup_out_pt_arena(base, op1, !discard_left);
            }
        } else {
            while (*(*op1).next).pt.x >= pt.x
                && (*(*op1).next).pt.x <= (*op1).pt.x
                && (*(*op1).next).pt.y == pt.y
            {
                op1 = (*op1).next;
            }
            if !discard_left && (*op1).pt.x != pt.x {
                op1 = (*op1).next;
            }
            op1b = dup_out_pt_arena(base, op1, discard_left);
            if (*op1b).pt != pt {
                op1 = op1b;
                (*op1).pt = pt;
                op1b = dup_out_pt_arena(base, op1, discard_left);
            }
        }

        if dir2 == Direction::LeftToRight {
            while (*(*op2).next).pt.x <= pt.x
                && (*(*op2).next).pt.x >= (*op2).pt.x
                && (*(*op2).next).pt.y == pt.y
            {
                op2 = (*op2).next;
            }
            if discard_left && (*op2).pt.x != pt.x {
                op2 = (*op2).next;
            }
            op2b = dup_out_pt_arena(base, op2, !discard_left);
            if (*op2b).pt != pt {
                op2 = op2b;
                (*op2).pt = pt;
                op2b = dup_out_pt_arena(base, op2, !discard_left);
            }
        } else {
            while (*(*op2).next).pt.x >= pt.x
                && (*(*op2).next).pt.x <= (*op2).pt.x
                && (*(*op2).next).pt.y == pt.y
            {
                op2 = (*op2).next;
            }
            if !discard_left && (*op2).pt.x != pt.x {
                op2 = (*op2).next;
            }
            op2b = dup_out_pt_arena(base, op2, discard_left);
            if (*op2b).pt != pt {
                op2 = op2b;
                (*op2).pt = pt;
                op2b = dup_out_pt_arena(base, op2, discard_left);
            }
        }

        if (dir1 == Direction::LeftToRight) == discard_left {
            (*op1).prev = op2;
            (*op2).next = op1;
            (*op1b).next = op2b;
            (*op2b).prev = op1b;
        } else {
            (*op1).next = op2;
            (*op2).prev = op1;
            (*op1b).prev = op2b;
            (*op2b).next = op1b;
        }
    }
    true
}

// C++: ParseFirstLeft
unsafe fn parse_first_left(mut first_left: *mut OutRec) -> *mut OutRec {
    unsafe {
        while !first_left.is_null() && (*first_left).pts.is_null() {
            first_left = (*first_left).first_left;
        }
    }
    first_left
}

// C++: GetHorzDirection
#[allow(dead_code)]
fn get_horz_direction(horz_edge: &TEdge) -> (Direction, CInt, CInt) {
    if horz_edge.bot.x < horz_edge.top.x {
        (Direction::LeftToRight, horz_edge.bot.x, horz_edge.top.x)
    } else {
        (Direction::RightToLeft, horz_edge.top.x, horz_edge.bot.x)
    }
}

fn winding_count_for_fill(fill_type: PolyFillType, wind_cnt: i32) -> i32 {
    match fill_type {
        PolyFillType::Positive => wind_cnt,
        PolyFillType::Negative => -wind_cnt,
        PolyFillType::EvenOdd | PolyFillType::NonZero => wind_cnt.abs(),
    }
}

// C++: EdgesAdjacent
unsafe fn edges_adjacent(inode: &IntersectNode) -> bool {
    unsafe {
        (*inode.edge1).next_in_sel == inode.edge2 || (*inode.edge1).prev_in_sel == inode.edge2
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{OutPt, OutRec};

    #[test]
    fn constructor_applies_init_option_bits() {
        let clipper = Clipper::with_init_options(
            InitOptions::ReverseSolution as i32
                | InitOptions::StrictlySimple as i32
                | InitOptions::PreserveCollinear as i32,
        );

        assert!(clipper.reverse_solution());
        assert!(clipper.strictly_simple());
        assert!(clipper.base.preserve_collinear());
        assert!(!clipper.execute_locked);
        assert!(!clipper.base.has_open_paths);
    }

    #[test]
    fn setters_match_cpp_properties() {
        let mut clipper = Clipper::new();

        clipper.set_reverse_solution(true);
        clipper.set_strictly_simple(true);

        assert!(clipper.reverse_solution());
        assert!(clipper.strictly_simple());
    }

    #[test]
    fn execute_rejects_open_paths_for_paths_result() {
        let mut clipper = Clipper::new();
        clipper
            .add_path(
                &vec![IntPoint::new(0, 10), IntPoint::new(10, 0)],
                PolyType::Subject,
                false,
            )
            .unwrap();
        let mut solution = Vec::new();

        let err = clipper
            .execute(ClipType::Union, &mut solution, PolyFillType::EvenOdd)
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Error: PolyTree struct is needed for open path clipping."
        );
    }

    #[test]
    fn execute_polytree_builds_closed_polygon_node() {
        let mut clipper = Clipper::new();
        clipper
            .add_path(
                &vec![
                    IntPoint::new(0, 0),
                    IntPoint::new(10, 0),
                    IntPoint::new(10, 10),
                    IntPoint::new(0, 10),
                ],
                PolyType::Subject,
                true,
            )
            .unwrap();
        let mut polytree = PolyTree::new();

        let succeeded = clipper
            .execute_polytree(ClipType::Union, &mut polytree, PolyFillType::EvenOdd)
            .unwrap();

        assert!(succeeded);
        assert_eq!(polytree.total(), 1);
        unsafe {
            assert_eq!((*polytree.get_first()).contour.len(), 4);
            assert!(!(*polytree.get_first()).is_open());
        }
    }

    #[test]
    fn execute_polytree_builds_open_path_node() {
        let mut clipper = Clipper::new();
        clipper
            .add_path(
                &vec![IntPoint::new(0, 10), IntPoint::new(10, 0)],
                PolyType::Subject,
                false,
            )
            .unwrap();
        let mut polytree = PolyTree::new();

        let succeeded = clipper
            .execute_polytree(ClipType::Union, &mut polytree, PolyFillType::EvenOdd)
            .unwrap();

        assert!(succeeded);
        assert_eq!(polytree.total(), 1);
        let first = polytree.get_first();
        unsafe {
            assert!((*first).is_open());
            assert_eq!(
                (*first).contour,
                vec![IntPoint::new(0, 10), IntPoint::new(10, 0)]
            );
        }
    }

    #[test]
    fn execute_builds_solution_for_simple_closed_path() {
        let mut clipper = Clipper::new();
        clipper
            .add_path(
                &vec![
                    IntPoint::new(0, 0),
                    IntPoint::new(10, 0),
                    IntPoint::new(10, 10),
                    IntPoint::new(0, 10),
                ],
                PolyType::Subject,
                true,
            )
            .unwrap();
        let mut solution = vec![vec![IntPoint::new(1, 1)]];

        let succeeded = clipper
            .execute(ClipType::Union, &mut solution, PolyFillType::EvenOdd)
            .unwrap();

        assert!(succeeded);
        assert_eq!(solution.len(), 1);
        assert_eq!(solution[0].len(), 4);
        assert!(solution[0].contains(&IntPoint::new(0, 0)));
        assert!(solution[0].contains(&IntPoint::new(10, 0)));
        assert!(solution[0].contains(&IntPoint::new(10, 10)));
        assert!(solution[0].contains(&IntPoint::new(0, 10)));
        assert!(!clipper.execute_locked);
        assert_eq!(clipper.clip_type, ClipType::Union);
    }

    #[test]
    fn execute_unions_overlapping_rectangles() {
        let mut clipper = Clipper::new();
        clipper
            .add_path(
                &vec![
                    IntPoint::new(0, 0),
                    IntPoint::new(10, 0),
                    IntPoint::new(10, 10),
                    IntPoint::new(0, 10),
                ],
                PolyType::Subject,
                true,
            )
            .unwrap();
        clipper
            .add_path(
                &vec![
                    IntPoint::new(5, 5),
                    IntPoint::new(15, 5),
                    IntPoint::new(15, 15),
                    IntPoint::new(5, 15),
                ],
                PolyType::Subject,
                true,
            )
            .unwrap();
        let mut solution = Vec::new();

        let succeeded = clipper
            .execute(ClipType::Union, &mut solution, PolyFillType::NonZero)
            .unwrap();

        assert!(succeeded);
        assert_eq!(solution.len(), 1);
        assert_eq!(crate::helpers::area(&solution[0]).abs(), 175.0);
    }

    #[test]
    fn execute_strict_simple_no_longer_hits_unported_boundary() {
        let mut clipper = Clipper::with_init_options(InitOptions::StrictlySimple as i32);
        clipper
            .add_path(
                &vec![
                    IntPoint::new(0, 0),
                    IntPoint::new(10, 0),
                    IntPoint::new(10, 10),
                    IntPoint::new(0, 10),
                ],
                PolyType::Subject,
                true,
            )
            .unwrap();
        let mut solution = Vec::new();

        let succeeded = clipper
            .execute(ClipType::Union, &mut solution, PolyFillType::EvenOdd)
            .unwrap();

        assert!(succeeded);
        assert_eq!(solution.len(), 1);
    }

    #[test]
    fn fill_type_helpers_use_edge_poly_type() {
        let mut clipper = Clipper::new();
        clipper.subj_fill_type = PolyFillType::EvenOdd;
        clipper.clip_fill_type = PolyFillType::NonZero;

        let subject_edge = TEdge {
            poly_typ: PolyType::Subject,
            ..TEdge::default()
        };
        let clip_edge = TEdge {
            poly_typ: PolyType::Clip,
            ..TEdge::default()
        };

        assert!(clipper.is_even_odd_fill_type(&subject_edge));
        assert!(!clipper.is_even_odd_fill_type(&clip_edge));
        assert!(!clipper.is_even_odd_alt_fill_type(&subject_edge));
        assert!(clipper.is_even_odd_alt_fill_type(&clip_edge));
    }

    #[test]
    fn is_contributing_matches_union_and_intersection_cases() {
        let mut clipper = Clipper::new();
        clipper.subj_fill_type = PolyFillType::NonZero;
        clipper.clip_fill_type = PolyFillType::NonZero;

        let mut edge = TEdge {
            poly_typ: PolyType::Subject,
            wind_delta: 1,
            wind_cnt: 1,
            wind_cnt2: 0,
            ..TEdge::default()
        };

        clipper.clip_type = ClipType::Union;
        assert!(clipper.is_contributing(&edge));

        edge.wind_cnt2 = 1;
        assert!(!clipper.is_contributing(&edge));

        clipper.clip_type = ClipType::Intersection;
        assert!(clipper.is_contributing(&edge));
    }

    #[test]
    fn set_winding_count_initializes_first_edge() {
        let mut clipper = Clipper::new();
        clipper.subj_fill_type = PolyFillType::Negative;
        let mut edge = TEdge {
            poly_typ: PolyType::Subject,
            wind_delta: 0,
            ..TEdge::default()
        };
        let edge_ptr = &mut edge as *mut TEdge;
        clipper.base.active_edges = edge_ptr;

        unsafe {
            clipper.set_winding_count(edge_ptr);
        }

        assert_eq!(edge.wind_cnt, -1);
        assert_eq!(edge.wind_cnt2, 0);
    }

    #[test]
    fn set_winding_count_uses_previous_same_poly_edge() {
        let mut clipper = Clipper::new();
        clipper.subj_fill_type = PolyFillType::NonZero;
        clipper.clip_fill_type = PolyFillType::NonZero;
        clipper.clip_type = ClipType::Union;

        let mut edges = vec![TEdge::default(), TEdge::default(), TEdge::default()];
        let e0 = edges.as_mut_ptr();
        let e1 = unsafe { e0.add(1) };
        let e2 = unsafe { e0.add(2) };

        unsafe {
            (*e0).poly_typ = PolyType::Subject;
            (*e0).wind_delta = 1;
            (*e0).wind_cnt = 1;
            (*e0).wind_cnt2 = 0;
            (*e0).next_in_ael = e1;

            (*e1).poly_typ = PolyType::Clip;
            (*e1).wind_delta = 1;
            (*e1).next_in_ael = e2;
            (*e1).prev_in_ael = e0;

            (*e2).poly_typ = PolyType::Subject;
            (*e2).wind_delta = 1;
            (*e2).prev_in_ael = e1;
            clipper.base.active_edges = e0;

            clipper.set_winding_count(e2);

            assert_eq!((*e2).wind_cnt, 2);
            assert_eq!((*e2).wind_cnt2, 1);
        }
    }

    #[test]
    fn joins_and_intersect_nodes_are_owned_and_cleared() {
        let mut clipper = Clipper::new();
        let op = Box::into_raw(Box::new(OutPt::default()));
        clipper.add_join(op, ptr::null_mut(), IntPoint::new(1, 2));
        clipper.add_ghost_join(op, IntPoint::new(3, 4));
        clipper.intersect_list.push(IntersectNode::default());

        unsafe {
            assert_eq!(clipper.joins[0].off_pt, IntPoint::new(1, 2));
            assert_eq!(clipper.ghost_joins[0].off_pt, IntPoint::new(3, 4));
            clipper.clear_joins();
            clipper.clear_ghost_joins();
            clipper.dispose_intersect_nodes();
            drop(Box::from_raw(op));
        }

        assert!(clipper.joins.is_empty());
        assert!(clipper.ghost_joins.is_empty());
        assert!(clipper.intersect_list.is_empty());
    }

    #[test]
    fn fix_hole_linkage_skips_same_hole_or_empty_owners() {
        let mut clipper = Clipper::new();
        let mut outer = Box::new(OutRec {
            is_hole: false,
            pts: std::ptr::dangling_mut(),
            ..OutRec::default()
        });
        let mut empty_same = Box::new(OutRec {
            is_hole: true,
            first_left: &mut *outer,
            ..OutRec::default()
        });
        let mut outrec = OutRec {
            is_hole: true,
            first_left: &mut *empty_same,
            ..OutRec::default()
        };
        let outer_ptr = &mut *outer as *mut OutRec;

        unsafe {
            clipper.fix_hole_linkage(&mut outrec);
        }

        assert_eq!(outrec.first_left, outer_ptr);
    }

    #[test]
    fn add_out_pt_creates_and_extends_output_ring() {
        let mut clipper = Clipper::new();
        let mut edge = TEdge {
            out_idx: UNASSIGNED,
            side: EdgeSide::Left,
            wind_delta: 1,
            ..TEdge::default()
        };
        let edge_ptr = &mut edge as *mut TEdge;

        unsafe {
            let first = clipper.add_out_pt(edge_ptr, IntPoint::new(0, 0));
            assert_eq!(edge.out_idx, 0);
            assert_eq!((*first).next, first);
            assert_eq!((*first).prev, first);
            assert_eq!((*clipper.base.poly_outs[0]).pts, first);

            let second = clipper.add_out_pt(edge_ptr, IntPoint::new(1, 0));
            assert_eq!((*clipper.base.poly_outs[0]).pts, second);
            assert_eq!((*second).next, first);
            assert_eq!((*first).prev, second);
            assert_eq!(clipper.get_last_out_pt(edge_ptr), second);
        }
    }

    #[test]
    fn add_out_pt_right_side_appends_at_ring_back() {
        let mut clipper = Clipper::new();
        let mut edge = TEdge {
            out_idx: UNASSIGNED,
            side: EdgeSide::Right,
            wind_delta: 1,
            ..TEdge::default()
        };
        let edge_ptr = &mut edge as *mut TEdge;

        unsafe {
            let first = clipper.add_out_pt(edge_ptr, IntPoint::new(0, 0));
            let second = clipper.add_out_pt(edge_ptr, IntPoint::new(1, 0));
            assert_eq!((*clipper.base.poly_outs[0]).pts, first);
            assert_eq!((*first).prev, second);
            assert_eq!(clipper.get_last_out_pt(edge_ptr), second);
        }
    }

    #[test]
    fn set_hole_state_uses_previous_output_edge() {
        let mut clipper = Clipper::new();
        let owner = clipper.base.create_out_rec();
        unsafe {
            (*owner).is_hole = false;
            let owner_pt = Box::into_raw(Box::new(OutPt {
                idx: 0,
                pt: IntPoint::new(0, 0),
                next: ptr::null_mut(),
                prev: ptr::null_mut(),
            }));
            (*owner_pt).next = owner_pt;
            (*owner_pt).prev = owner_pt;
            (*owner).pts = owner_pt;
        }

        let mut prev = TEdge {
            out_idx: 0,
            wind_delta: 1,
            ..TEdge::default()
        };
        let mut edge = TEdge {
            prev_in_ael: &mut prev,
            ..TEdge::default()
        };
        let mut child = OutRec::default();

        unsafe {
            clipper.set_hole_state(&mut edge, &mut child);
        }

        assert_eq!(child.first_left, owner);
        assert!(child.is_hole);
    }

    #[test]
    fn add_local_min_poly_assigns_shared_out_idx_and_sides() {
        let mut clipper = Clipper::new();
        let mut e1 = TEdge {
            out_idx: UNASSIGNED,
            dx: 2.0,
            wind_delta: 1,
            ..TEdge::default()
        };
        let mut e2 = TEdge {
            out_idx: UNASSIGNED,
            dx: 1.0,
            wind_delta: -1,
            ..TEdge::default()
        };

        unsafe {
            let out_pt = clipper.add_local_min_poly(&mut e1, &mut e2, IntPoint::new(5, 5));

            assert!(!out_pt.is_null());
            assert_eq!(e1.out_idx, e2.out_idx);
            assert_eq!(e1.side, EdgeSide::Left);
            assert_eq!(e2.side, EdgeSide::Right);
            assert_eq!((*out_pt).pt, IntPoint::new(5, 5));
        }
    }

    #[test]
    fn add_local_max_poly_closes_same_output_record() {
        let mut clipper = Clipper::new();
        let mut e1 = TEdge {
            out_idx: UNASSIGNED,
            side: EdgeSide::Left,
            wind_delta: 1,
            ..TEdge::default()
        };
        let mut e2 = TEdge {
            out_idx: 0,
            side: EdgeSide::Right,
            wind_delta: -1,
            ..TEdge::default()
        };

        unsafe {
            clipper.add_out_pt(&mut e1, IntPoint::new(0, 0));
            e2.out_idx = e1.out_idx;
            clipper.add_local_max_poly(&mut e1, &mut e2, IntPoint::new(1, 1));
        }

        assert_eq!(e1.out_idx, UNASSIGNED);
        assert_eq!(e2.out_idx, UNASSIGNED);
    }

    #[test]
    fn intersect_edges_swaps_contributing_same_poly_edges_for_xor_style_case() {
        let mut clipper = Clipper::new();
        clipper.clip_type = ClipType::Xor;
        clipper.subj_fill_type = PolyFillType::EvenOdd;
        clipper.clip_fill_type = PolyFillType::EvenOdd;

        let mut e1 = TEdge {
            out_idx: UNASSIGNED,
            poly_typ: PolyType::Subject,
            wind_delta: 1,
            wind_cnt: 1,
            side: EdgeSide::Left,
            ..TEdge::default()
        };
        let mut e2 = TEdge {
            out_idx: UNASSIGNED,
            poly_typ: PolyType::Subject,
            wind_delta: -1,
            wind_cnt: 1,
            side: EdgeSide::Right,
            ..TEdge::default()
        };

        unsafe {
            clipper.add_out_pt(&mut e1, IntPoint::new(0, 0));
            clipper.add_out_pt(&mut e2, IntPoint::new(10, 0));
            let e1_idx = e1.out_idx;
            let e2_idx = e2.out_idx;

            clipper.intersect_edges(&mut e1, &mut e2, IntPoint::new(5, 5));

            assert_eq!(e1.side, EdgeSide::Right);
            assert_eq!(e2.side, EdgeSide::Left);
            assert_eq!(e1.out_idx, e2_idx);
            assert_eq!(e2.out_idx, e1_idx);
        }
    }

    #[test]
    fn intersect_edges_toggles_open_subject_output_against_clip_edge() {
        let mut clipper = Clipper::new();
        clipper.clip_type = ClipType::Difference;
        clipper.subj_fill_type = PolyFillType::EvenOdd;
        clipper.clip_fill_type = PolyFillType::EvenOdd;

        let mut open = TEdge {
            out_idx: UNASSIGNED,
            poly_typ: PolyType::Subject,
            wind_delta: 0,
            side: EdgeSide::Left,
            ..TEdge::default()
        };
        let mut clip = TEdge {
            out_idx: UNASSIGNED,
            poly_typ: PolyType::Clip,
            wind_delta: 1,
            wind_cnt: 1,
            ..TEdge::default()
        };

        unsafe {
            clipper.intersect_edges(&mut open, &mut clip, IntPoint::new(2, 2));
            assert_eq!(clipper.base.poly_outs.len(), 1);
            assert_eq!(open.out_idx, 0);
            assert!((*clipper.base.poly_outs[0]).is_open);
        }
    }

    #[test]
    fn sel_push_pop_and_delete_match_stack_style_order() {
        let mut clipper = Clipper::new();
        let mut edges = vec![TEdge::default(), TEdge::default()];
        let e0 = edges.as_mut_ptr();
        let e1 = unsafe { e0.add(1) };

        unsafe {
            clipper.add_edge_to_sel(e0);
            clipper.add_edge_to_sel(e1);

            assert_eq!(clipper.sorted_edges, e1);
            assert_eq!((*e1).next_in_sel, e0);
            assert_eq!((*e0).prev_in_sel, e1);

            assert_eq!(clipper.pop_edge_from_sel(), Some(e1));
            assert_eq!(clipper.pop_edge_from_sel(), Some(e0));
            assert_eq!(clipper.pop_edge_from_sel(), None);
        }
    }

    #[test]
    fn copy_ael_to_sel_and_swap_positions_in_sel_preserve_links() {
        let mut clipper = Clipper::new();
        let mut edges = vec![TEdge::default(), TEdge::default(), TEdge::default()];
        let e0 = edges.as_mut_ptr();
        let e1 = unsafe { e0.add(1) };
        let e2 = unsafe { e0.add(2) };

        unsafe {
            (*e0).next_in_ael = e1;
            (*e1).prev_in_ael = e0;
            (*e1).next_in_ael = e2;
            (*e2).prev_in_ael = e1;
            clipper.base.active_edges = e0;

            clipper.copy_ael_to_sel();
            assert_eq!(clipper.sorted_edges, e0);
            assert_eq!((*e1).prev_in_sel, e0);
            assert_eq!((*e1).next_in_sel, e2);

            clipper.swap_positions_in_sel(e0, e1);
            assert_eq!(clipper.sorted_edges, e1);
            assert_eq!((*e1).next_in_sel, e0);
            assert_eq!((*e0).prev_in_sel, e1);
            assert_eq!((*e0).next_in_sel, e2);
            assert_eq!((*e2).prev_in_sel, e0);
        }
    }

    #[test]
    fn insert_edge_into_ael_orders_by_current_x() {
        let mut clipper = Clipper::new();
        let mut left = TEdge {
            curr: IntPoint::new(0, 0),
            top: IntPoint::new(0, 10),
            ..TEdge::default()
        };
        let mut right = TEdge {
            curr: IntPoint::new(10, 0),
            top: IntPoint::new(10, 10),
            ..TEdge::default()
        };
        let left_ptr = &mut left as *mut TEdge;
        let right_ptr = &mut right as *mut TEdge;

        unsafe {
            clipper.insert_edge_into_ael(right_ptr, ptr::null_mut());
            clipper.insert_edge_into_ael(left_ptr, ptr::null_mut());

            assert_eq!(clipper.base.active_edges, left_ptr);
            assert_eq!((*left_ptr).next_in_ael, right_ptr);
            assert_eq!((*right_ptr).prev_in_ael, left_ptr);
        }
    }

    #[test]
    fn ael_direction_and_horizontal_direction_helpers_match_cpp() {
        let mut edges = vec![TEdge::default(), TEdge::default(), TEdge::default()];
        let e0 = edges.as_mut_ptr();
        let e1 = unsafe { e0.add(1) };
        let e2 = unsafe { e0.add(2) };

        unsafe {
            (*e1).prev_in_ael = e0;
            (*e1).next_in_ael = e2;
            assert_eq!(get_next_in_ael(e1, Direction::LeftToRight), e2);
            assert_eq!(get_next_in_ael(e1, Direction::RightToLeft), e0);
        }

        let edge = TEdge {
            bot: IntPoint::new(10, 0),
            top: IntPoint::new(0, 0),
            ..TEdge::default()
        };
        assert_eq!(get_horz_direction(&edge), (Direction::RightToLeft, 0, 10));
    }

    #[test]
    fn maxima_predicates_and_pair_helpers_match_cpp() {
        let mut edges = vec![TEdge::default(), TEdge::default(), TEdge::default()];
        let e0 = edges.as_mut_ptr();
        let e1 = unsafe { e0.add(1) };
        let e2 = unsafe { e0.add(2) };

        unsafe {
            (*e0).next = e1;
            (*e1).prev = e0;
            (*e1).next = e2;
            (*e2).prev = e1;
            (*e2).next = e0;
            (*e0).prev = e2;

            (*e1).top = IntPoint::new(10, 20);
            (*e2).top = IntPoint::new(10, 20);
            (*e1).next_in_lml = ptr::null_mut();
            (*e2).next_in_lml = ptr::null_mut();
            (*e2).next_in_ael = e0;

            assert!(is_minima(e1));
            assert!(is_maxima(e1, 20));
            assert!(!is_maxima(e1, 19));
            assert_eq!(get_maxima_pair(e1), e2);
            assert_eq!(get_maxima_pair_ex(e1), e2);

            (*e2).out_idx = SKIP;
            assert!(get_maxima_pair_ex(e1).is_null());

            (*e2).out_idx = UNASSIGNED;
            (*e1).next_in_lml = e0;
            assert!(is_intermediate(e1, 20));
        }
    }

    #[test]
    fn do_maxima_without_pair_adds_top_and_deletes_from_ael() {
        let mut clipper = Clipper::new();
        let mut edges = vec![TEdge::default(), TEdge::default(), TEdge::default()];
        let e0 = edges.as_mut_ptr();
        let e1 = unsafe { e0.add(1) };
        let e2 = unsafe { e0.add(2) };

        unsafe {
            (*e0).next = e1;
            (*e1).prev = e0;
            (*e1).next = e2;
            (*e2).prev = e1;
            (*e2).next = e0;
            (*e0).prev = e2;

            (*e1).top = IntPoint::new(3, 7);
            (*e2).top = IntPoint::new(30, 70);
            (*e1).out_idx = UNASSIGNED;
            (*e1).side = EdgeSide::Left;
            (*e1).wind_delta = 1;
            clipper.add_out_pt(e1, IntPoint::new(1, 1));

            clipper.base.active_edges = e1;
            clipper.do_maxima(e1).unwrap();

            assert!(clipper.base.active_edges.is_null());
            let out_rec = clipper.base.poly_outs[0];
            assert_eq!((*(*out_rec).pts).pt, IntPoint::new(3, 7));
        }
    }

    #[test]
    fn do_maxima_deletes_unassigned_maxima_pair() {
        let mut clipper = Clipper::new();
        let mut edges = vec![TEdge::default(), TEdge::default()];
        let e0 = edges.as_mut_ptr();
        let e1 = unsafe { e0.add(1) };

        unsafe {
            (*e0).next = e1;
            (*e0).prev = e1;
            (*e1).next = e0;
            (*e1).prev = e0;
            (*e0).top = IntPoint::new(5, 9);
            (*e1).top = IntPoint::new(5, 9);
            (*e0).out_idx = UNASSIGNED;
            (*e1).out_idx = UNASSIGNED;
            (*e0).next_in_ael = e1;
            (*e1).prev_in_ael = e0;
            clipper.base.active_edges = e0;

            clipper.do_maxima(e0).unwrap();

            assert!(clipper.base.active_edges.is_null());
            assert!((*e0).next_in_ael.is_null());
            assert!((*e1).prev_in_ael.is_null());
        }
    }

    #[test]
    fn do_maxima_reports_cpp_error_when_only_one_closed_edge_has_output() {
        let mut clipper = Clipper::new();
        let mut edges = vec![TEdge::default(), TEdge::default()];
        let e0 = edges.as_mut_ptr();
        let e1 = unsafe { e0.add(1) };

        unsafe {
            (*e0).next = e1;
            (*e0).prev = e1;
            (*e1).next = e0;
            (*e1).prev = e0;
            (*e0).top = IntPoint::new(5, 9);
            (*e1).top = IntPoint::new(5, 9);
            (*e0).out_idx = 0;
            (*e0).wind_delta = 1;
            (*e1).out_idx = UNASSIGNED;
            (*e1).wind_delta = -1;
            (*e0).next_in_ael = e1;
            (*e1).prev_in_ael = e0;
            clipper.base.active_edges = e0;

            let err = clipper.do_maxima(e0).unwrap_err();

            assert_eq!(err.to_string(), "DoMaxima error");
        }
    }

    #[test]
    fn process_horizontals_processes_and_removes_terminal_horizontal_edge() {
        let mut clipper = Clipper::new();
        let mut edge = TEdge {
            bot: IntPoint::new(0, 0),
            curr: IntPoint::new(0, 0),
            top: IntPoint::new(10, 0),
            out_idx: UNASSIGNED,
            wind_delta: 0,
            ..TEdge::default()
        };
        let mut prev = TEdge {
            top: IntPoint::new(-10, 0),
            ..TEdge::default()
        };
        let mut next = TEdge {
            top: IntPoint::new(20, 0),
            ..TEdge::default()
        };
        let edge_ptr = &mut edge as *mut TEdge;
        let prev_ptr = &mut prev as *mut TEdge;
        let next_ptr = &mut next as *mut TEdge;

        unsafe {
            (*edge_ptr).prev = prev_ptr;
            (*edge_ptr).next = next_ptr;
            (*prev_ptr).next = edge_ptr;
            (*next_ptr).prev = edge_ptr;
            clipper.base.active_edges = edge_ptr;
            clipper.add_edge_to_sel(edge_ptr);
            clipper.process_horizontals().unwrap();

            assert!(clipper.sorted_edges.is_null());
            assert!(clipper.base.active_edges.is_null());
        }
    }

    #[test]
    fn process_edges_at_top_updates_non_maxima_current_point() {
        let mut clipper = Clipper::new();
        let mut edge = TEdge {
            bot: IntPoint::new(0, 10),
            curr: IntPoint::new(0, 10),
            top: IntPoint::new(10, 0),
            dx: -1.0,
            ..TEdge::default()
        };
        let edge_ptr = &mut edge as *mut TEdge;
        clipper.base.active_edges = edge_ptr;

        unsafe {
            clipper.process_edges_at_top_of_scanbeam(5).unwrap();
        }

        assert_eq!(edge.curr, IntPoint::new(5, 5));
    }

    #[test]
    fn process_edges_at_top_promotes_intermediate_edge() {
        let mut clipper = Clipper::new();
        let mut edge = TEdge {
            bot: IntPoint::new(0, 10),
            curr: IntPoint::new(0, 10),
            top: IntPoint::new(0, 5),
            out_idx: UNASSIGNED,
            wind_delta: 1,
            ..TEdge::default()
        };
        let mut next = TEdge {
            bot: IntPoint::new(0, 5),
            curr: IntPoint::new(0, 5),
            top: IntPoint::new(5, 0),
            dx: -1.0,
            ..TEdge::default()
        };
        edge.next_in_lml = &mut next;
        let edge_ptr = &mut edge as *mut TEdge;
        let next_ptr = &mut next as *mut TEdge;
        clipper.base.active_edges = edge_ptr;

        unsafe {
            clipper.process_edges_at_top_of_scanbeam(5).unwrap();
        }

        assert_eq!(clipper.base.active_edges, next_ptr);
        assert_eq!(next.curr, next.bot);
        assert_eq!(clipper.base.pop_scanbeam(), Some(0));
    }

    unsafe fn add_outrec_with_ring(
        clipper: &mut Clipper,
        pts: &[IntPoint],
        is_open: bool,
    ) -> *mut OutRec {
        unsafe {
            let outrec = clipper.base.create_out_rec();
            (*outrec).is_open = is_open;
            let mut raw_pts: Vec<*mut OutPt> = Vec::with_capacity(pts.len());
            for pt in pts {
                raw_pts.push(Box::into_raw(Box::new(OutPt {
                    idx: (*outrec).idx,
                    pt: *pt,
                    next: ptr::null_mut(),
                    prev: ptr::null_mut(),
                })));
            }
            for i in 0..raw_pts.len() {
                (*raw_pts[i]).next = raw_pts[(i + 1) % raw_pts.len()];
                (*raw_pts[i]).prev = raw_pts[(i + raw_pts.len() - 1) % raw_pts.len()];
            }
            (*outrec).pts = raw_pts[0];
            outrec
        }
    }

    #[test]
    fn point_count_and_build_result_walk_output_ring_backwards() {
        let mut clipper = Clipper::new();
        unsafe {
            let outrec = add_outrec_with_ring(
                &mut clipper,
                &[
                    IntPoint::new(0, 0),
                    IntPoint::new(10, 0),
                    IntPoint::new(10, 10),
                ],
                false,
            );

            assert_eq!(point_count((*outrec).pts), 3);

            let mut paths = Vec::new();
            clipper.build_result(&mut paths);

            assert_eq!(
                paths,
                vec![vec![
                    IntPoint::new(10, 10),
                    IntPoint::new(10, 0),
                    IntPoint::new(0, 0),
                ]]
            );
        }
    }

    #[test]
    fn fixup_out_polyline_removes_duplicate_points() {
        let mut clipper = Clipper::new();
        unsafe {
            let outrec = add_outrec_with_ring(
                &mut clipper,
                &[
                    IntPoint::new(0, 0),
                    IntPoint::new(1, 1),
                    IntPoint::new(1, 1),
                ],
                true,
            );

            clipper.fixup_out_polyline(outrec);

            assert_eq!(point_count((*outrec).pts), 2);
        }
    }

    #[test]
    fn fixup_out_polygon_removes_collinear_middle_vertex() {
        let mut clipper = Clipper::new();
        unsafe {
            let outrec = add_outrec_with_ring(
                &mut clipper,
                &[
                    IntPoint::new(0, 0),
                    IntPoint::new(5, 0),
                    IntPoint::new(10, 0),
                    IntPoint::new(10, 10),
                    IntPoint::new(0, 10),
                ],
                false,
            );

            clipper.fixup_out_polygon(outrec);

            assert_eq!(point_count((*outrec).pts), 4);
        }
    }

    #[test]
    fn build_result2_links_holes_and_open_paths_into_polytree() {
        let mut clipper = Clipper::new();
        let mut polytree = PolyTree::new();
        unsafe {
            let outer = add_outrec_with_ring(
                &mut clipper,
                &[
                    IntPoint::new(0, 0),
                    IntPoint::new(10, 0),
                    IntPoint::new(10, 10),
                ],
                false,
            );
            let hole = add_outrec_with_ring(
                &mut clipper,
                &[
                    IntPoint::new(2, 2),
                    IntPoint::new(4, 2),
                    IntPoint::new(4, 4),
                ],
                false,
            );
            let open = add_outrec_with_ring(
                &mut clipper,
                &[IntPoint::new(20, 20), IntPoint::new(30, 30)],
                true,
            );
            (*outer).is_hole = false;
            (*hole).is_hole = true;
            (*hole).first_left = outer;

            clipper.build_result2(&mut polytree);

            assert_eq!(polytree.all_nodes.len(), 3);
            assert_eq!(polytree.node.child_count(), 2);
            assert_eq!((*(*outer).poly_nd).child_count(), 1);
            assert!((*(*open).poly_nd).is_open());
            assert_eq!((*(*hole).poly_nd).parent, (*outer).poly_nd);
        }
    }

    #[test]
    fn update_out_pt_idxs_relabels_whole_output_ring() {
        let mut clipper = Clipper::new();
        unsafe {
            let outrec = add_outrec_with_ring(
                &mut clipper,
                &[
                    IntPoint::new(0, 0),
                    IntPoint::new(10, 0),
                    IntPoint::new(10, 10),
                ],
                false,
            );
            (*outrec).idx = 7;

            update_out_pt_idxs(outrec);

            let mut op = (*outrec).pts;
            for _ in 0..3 {
                assert_eq!((*op).idx, 7);
                op = (*op).next;
            }
            assert_eq!(op, (*outrec).pts);
        }
    }

    #[test]
    fn dup_out_pt_inserts_before_or_after_existing_point() {
        let mut clipper = Clipper::new();
        unsafe {
            let outrec = add_outrec_with_ring(
                &mut clipper,
                &[
                    IntPoint::new(0, 0),
                    IntPoint::new(10, 0),
                    IntPoint::new(10, 10),
                ],
                false,
            );
            let first = (*outrec).pts;

            let after = dup_out_pt(first, true);
            assert_eq!((*first).next, after);
            assert_eq!((*after).prev, first);
            assert_eq!((*after).pt, (*first).pt);

            let before = dup_out_pt(first, false);
            assert_eq!((*first).prev, before);
            assert_eq!((*before).next, first);
            assert_eq!(point_count(first), 5);
        }
    }

    #[test]
    fn fixup_first_lefts_reassign_output_owners() {
        let mut clipper = Clipper::new();
        unsafe {
            let old = add_outrec_with_ring(
                &mut clipper,
                &[
                    IntPoint::new(-10, -10),
                    IntPoint::new(20, -10),
                    IntPoint::new(20, 20),
                    IntPoint::new(-10, 20),
                ],
                false,
            );
            let new_owner = add_outrec_with_ring(
                &mut clipper,
                &[
                    IntPoint::new(0, 0),
                    IntPoint::new(10, 0),
                    IntPoint::new(10, 10),
                    IntPoint::new(0, 10),
                ],
                false,
            );
            let child = add_outrec_with_ring(
                &mut clipper,
                &[
                    IntPoint::new(2, 2),
                    IntPoint::new(4, 2),
                    IntPoint::new(4, 4),
                    IntPoint::new(2, 4),
                ],
                false,
            );
            (*child).first_left = old;

            clipper.fixup_first_lefts1(old, new_owner);
            assert_eq!((*child).first_left, new_owner);

            clipper.fixup_first_lefts3(new_owner, old);
            assert_eq!((*child).first_left, old);
        }
    }

    #[test]
    fn do_simple_polygons_splits_repeated_vertex_ring() {
        let mut clipper = Clipper::new();
        unsafe {
            add_outrec_with_ring(
                &mut clipper,
                &[
                    IntPoint::new(0, 0),
                    IntPoint::new(10, 0),
                    IntPoint::new(10, 10),
                    IntPoint::new(0, 0),
                    IntPoint::new(-10, 10),
                    IntPoint::new(-10, 0),
                ],
                false,
            );

            clipper.do_simple_polygons();

            assert_eq!(clipper.base.poly_outs.len(), 2);
            assert!(!clipper.base.poly_outs[1].is_null());
            assert!(!(*clipper.base.poly_outs[1]).pts.is_null());
            assert_eq!(point_count((*clipper.base.poly_outs[0]).pts), 3);
            assert_eq!(point_count((*clipper.base.poly_outs[1]).pts), 3);
        }
    }

    #[test]
    fn build_intersect_list_records_crossing_edges() {
        let mut clipper = Clipper::new();
        let mut e1 = TEdge {
            bot: IntPoint::new(0, 10),
            curr: IntPoint::new(0, 10),
            top: IntPoint::new(10, 0),
            dx: -1.0,
            ..TEdge::default()
        };
        let mut e2 = TEdge {
            bot: IntPoint::new(10, 10),
            curr: IntPoint::new(10, 10),
            top: IntPoint::new(0, 0),
            dx: 1.0,
            ..TEdge::default()
        };
        let e1_ptr = &mut e1 as *mut TEdge;
        let e2_ptr = &mut e2 as *mut TEdge;

        unsafe {
            (*e1_ptr).next_in_ael = e2_ptr;
            (*e2_ptr).prev_in_ael = e1_ptr;
            clipper.base.active_edges = e1_ptr;

            clipper.build_intersect_list(0);

            assert_eq!(clipper.intersect_list.len(), 1);
            let node = clipper.intersect_list[0];
            assert_eq!(node.edge1, e1_ptr);
            assert_eq!(node.edge2, e2_ptr);
            assert_eq!(node.pt, IntPoint::new(5, 5));
            assert!(clipper.sorted_edges.is_null());
        }
    }

    #[test]
    fn process_intersections_consumes_list_and_swaps_ael() {
        let mut clipper = Clipper::new();
        let mut e1 = TEdge {
            bot: IntPoint::new(0, 10),
            curr: IntPoint::new(0, 10),
            top: IntPoint::new(10, 0),
            dx: -1.0,
            wind_delta: 0,
            ..TEdge::default()
        };
        let mut e2 = TEdge {
            bot: IntPoint::new(10, 10),
            curr: IntPoint::new(10, 10),
            top: IntPoint::new(0, 0),
            dx: 1.0,
            wind_delta: 0,
            ..TEdge::default()
        };
        let e1_ptr = &mut e1 as *mut TEdge;
        let e2_ptr = &mut e2 as *mut TEdge;

        unsafe {
            (*e1_ptr).next_in_ael = e2_ptr;
            (*e2_ptr).prev_in_ael = e1_ptr;
            clipper.base.active_edges = e1_ptr;

            assert!(clipper.process_intersections(0).unwrap());

            assert!(clipper.intersect_list.is_empty());
            assert_eq!(clipper.base.active_edges, e2_ptr);
            assert_eq!((*e2_ptr).next_in_ael, e1_ptr);
            assert_eq!((*e1_ptr).prev_in_ael, e2_ptr);
            assert!(clipper.sorted_edges.is_null());
        }
    }

    #[test]
    fn insert_local_minima_into_ael_starts_closed_polygon_output() {
        let mut clipper = Clipper::new();
        let path = vec![
            IntPoint::new(0, 0),
            IntPoint::new(10, 0),
            IntPoint::new(10, 10),
            IntPoint::new(0, 10),
        ];
        clipper.add_path(&path, PolyType::Subject, true).unwrap();
        clipper.clip_type = ClipType::Union;
        clipper.subj_fill_type = PolyFillType::EvenOdd;
        clipper.clip_fill_type = PolyFillType::EvenOdd;

        unsafe {
            clipper.base.reset();
            let bot_y = clipper.base.pop_scanbeam().unwrap();
            clipper.insert_local_minima_into_ael(bot_y).unwrap();

            assert!(!clipper.base.active_edges.is_null());
            assert_eq!(clipper.base.poly_outs.len(), 1);
            let out_rec = clipper.base.poly_outs[0];
            assert!(!(*out_rec).pts.is_null());
            assert_eq!((*(*out_rec).pts).pt, IntPoint::new(0, 10));
            assert_eq!(clipper.base.pop_scanbeam(), Some(0));
        }
    }

    #[test]
    fn insert_local_minima_into_ael_adds_horizontal_open_edge_to_sel() {
        let mut clipper = Clipper::new();
        let path = vec![IntPoint::new(0, 0), IntPoint::new(10, 0)];
        clipper.add_path(&path, PolyType::Subject, false).unwrap();
        clipper.clip_type = ClipType::Union;
        clipper.subj_fill_type = PolyFillType::EvenOdd;
        clipper.clip_fill_type = PolyFillType::EvenOdd;

        unsafe {
            clipper.base.reset();
            let bot_y = clipper.base.pop_scanbeam().unwrap();
            clipper.insert_local_minima_into_ael(bot_y).unwrap();

            assert!(!clipper.base.active_edges.is_null());
            assert_eq!(clipper.sorted_edges, clipper.base.active_edges);
            assert_eq!(clipper.base.poly_outs.len(), 1);
            assert!((*clipper.base.poly_outs[0]).is_open);
        }
    }

    #[test]
    fn insert_local_minima_intersects_edges_between_left_and_right_bounds() {
        let mut clipper = Clipper::new();
        let mut lb = TEdge {
            out_idx: 0,
            wind_delta: 1,
            bot: IntPoint::new(0, 10),
            curr: IntPoint::new(0, 10),
            top: IntPoint::new(0, 0),
            ..TEdge::default()
        };
        let mut mid = TEdge {
            out_idx: 1,
            wind_delta: 1,
            curr: IntPoint::new(5, 10),
            top: IntPoint::new(5, 0),
            ..TEdge::default()
        };
        let mut rb = TEdge {
            out_idx: 2,
            wind_delta: -1,
            bot: IntPoint::new(10, 10),
            curr: IntPoint::new(10, 10),
            top: IntPoint::new(10, 0),
            ..TEdge::default()
        };
        let lb_ptr = &mut lb as *mut TEdge;
        let mid_ptr = &mut mid as *mut TEdge;
        let rb_ptr = &mut rb as *mut TEdge;

        unsafe {
            for i in 0..3 {
                let out_rec = clipper.base.create_out_rec();
                let op = Box::into_raw(Box::new(OutPt {
                    idx: i,
                    pt: IntPoint::new(i as CInt, 0),
                    next: ptr::null_mut(),
                    prev: ptr::null_mut(),
                }));
                (*op).next = op;
                (*op).prev = op;
                (*out_rec).pts = op;
            }

            clipper.base.active_edges = mid_ptr;
            clipper.base.minima_list.push(crate::types::LocalMinimum {
                y: 10,
                left_bound: lb_ptr,
                right_bound: rb_ptr,
            });

            clipper.insert_local_minima_into_ael(10).unwrap();
            assert!(!clipper.base.poly_outs.is_empty());
        }
    }
}
