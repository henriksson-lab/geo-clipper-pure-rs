use std::collections::BinaryHeap;
use std::fmt;
use std::mem::MaybeUninit;
use std::ptr;

use crate::error::{ClipperError, Result};
use crate::helpers::{
    init_edge, init_edge2, is_horizontal, pt2_is_between_pt1_and_pt3, range_test, remove_edge,
    reverse_horizontal, slopes_equal_3_points,
};
use crate::types::{
    CInt, EdgeSide, IntPoint, IntRect, LocalMinimum, OutPt, OutRec, Path, PolyType, SKIP, TEdge,
    UNASSIGNED,
};

const OUT_PT_BLOCK_SIZE: usize = 4096;

pub struct OutPtArena {
    blocks: Vec<Box<[MaybeUninit<OutPt>]>>,
    next: usize,
}

impl Default for OutPtArena {
    fn default() -> Self {
        Self {
            blocks: Vec::new(),
            next: OUT_PT_BLOCK_SIZE,
        }
    }
}

impl fmt::Debug for OutPtArena {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("OutPtArena")
            .field("blocks", &self.blocks.len())
            .field("next", &self.next)
            .finish()
    }
}

impl OutPtArena {
    pub fn alloc(&mut self, out_pt: OutPt) -> *mut OutPt {
        if self.next == OUT_PT_BLOCK_SIZE {
            let block = (0..OUT_PT_BLOCK_SIZE)
                .map(|_| MaybeUninit::uninit())
                .collect::<Vec<_>>()
                .into_boxed_slice();
            self.blocks.push(block);
            self.next = 0;
        }

        let block_index = self.blocks.len() - 1;
        // SAFETY: a block is pushed above when `next == OUT_PT_BLOCK_SIZE`, so
        // `block_index` exists and `next < OUT_PT_BLOCK_SIZE` here.
        let ptr = unsafe {
            self.blocks
                .get_unchecked_mut(block_index)
                .get_unchecked_mut(self.next)
                .as_mut_ptr()
        };
        self.next += 1;
        unsafe {
            ptr.write(out_pt);
        }
        ptr
    }

    pub fn clear(&mut self) {
        self.blocks.clear();
        self.next = OUT_PT_BLOCK_SIZE;
    }
}

#[derive(Debug)]
pub struct ClipperBase {
    pub current_lm: usize,
    pub minima_list: Vec<LocalMinimum>,
    pub use_full_range: bool,
    pub edges: Vec<Box<[TEdge]>>,
    pub preserve_collinear: bool,
    pub has_open_paths: bool,
    pub poly_outs: Vec<*mut OutRec>,
    pub out_rec_arena: Vec<Box<OutRec>>,
    pub out_pt_arena: OutPtArena,
    pub active_edges: *mut TEdge,
    pub scanbeam: BinaryHeap<CInt>,
}

impl Default for ClipperBase {
    fn default() -> Self {
        Self {
            current_lm: 0,
            minima_list: Vec::new(),
            use_full_range: false,
            edges: Vec::new(),
            preserve_collinear: false,
            has_open_paths: false,
            poly_outs: Vec::new(),
            out_rec_arena: Vec::new(),
            out_pt_arena: OutPtArena::default(),
            active_edges: ptr::null_mut(),
            scanbeam: BinaryHeap::new(),
        }
    }
}

impl ClipperBase {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn preserve_collinear(&self) -> bool {
        self.preserve_collinear
    }

    pub fn set_preserve_collinear(&mut self, value: bool) {
        self.preserve_collinear = value;
    }

    // C++: ClipperBase::AddPath
    pub fn add_path(&mut self, pg: &[IntPoint], poly_type: PolyType, closed: bool) -> Result<bool> {
        if !closed && poly_type == PolyType::Clip {
            return Err(ClipperError::new("AddPath: Open paths must be subject."));
        }

        if pg.is_empty() {
            return Ok(false);
        }

        let mut high_i = pg.len() - 1;
        if closed {
            while high_i > 0 && pg[high_i] == pg[0] {
                high_i -= 1;
            }
        }
        while high_i > 0 && pg[high_i] == pg[high_i - 1] {
            high_i -= 1;
        }
        if (closed && high_i < 2) || (!closed && high_i < 1) {
            return Ok(false);
        }

        let mut edges: Box<[TEdge]> = std::iter::repeat_with(TEdge::default)
            .take(high_i + 1)
            .collect::<Vec<_>>()
            .into_boxed_slice();
        let edges_ptr = edges.as_mut_ptr();
        let mut is_flat = true;

        unsafe {
            (*edges_ptr.add(1)).curr = pg[1];
        }
        range_test(pg[0], &mut self.use_full_range)?;
        range_test(pg[high_i], &mut self.use_full_range)?;
        unsafe {
            init_edge(edges_ptr, edges_ptr.add(1), edges_ptr.add(high_i), pg[0]);
            init_edge(
                edges_ptr.add(high_i),
                edges_ptr,
                edges_ptr.add(high_i - 1),
                pg[high_i],
            );
            for i in (1..high_i).rev() {
                range_test(pg[i], &mut self.use_full_range)?;
                init_edge(
                    edges_ptr.add(i),
                    edges_ptr.add(i + 1),
                    edges_ptr.add(i - 1),
                    pg[i],
                );
            }
        }

        let mut e_start = edges_ptr;

        unsafe {
            let mut e = e_start;
            let mut e_loop_stop = e_start;
            loop {
                if (*e).curr == (*(*e).next).curr && (closed || (*e).next != e_start) {
                    if e == (*e).next {
                        break;
                    }
                    if e == e_start {
                        e_start = (*e).next;
                    }
                    e = remove_edge(e);
                    e_loop_stop = e;
                    continue;
                }
                if (*e).prev == (*e).next {
                    break;
                } else if closed
                    && slopes_equal_3_points(
                        (*(*e).prev).curr,
                        (*e).curr,
                        (*(*e).next).curr,
                        self.use_full_range,
                    )
                    && (!self.preserve_collinear
                        || !pt2_is_between_pt1_and_pt3(
                            (*(*e).prev).curr,
                            (*e).curr,
                            (*(*e).next).curr,
                        ))
                {
                    if e == e_start {
                        e_start = (*e).next;
                    }
                    e = remove_edge(e);
                    e = (*e).prev;
                    e_loop_stop = e;
                    continue;
                }
                e = (*e).next;
                if e == e_loop_stop || (!closed && (*e).next == e_start) {
                    break;
                }
            }

            if (!closed && e == (*e).next) || (closed && (*e).prev == (*e).next) {
                return Ok(false);
            }

            if !closed {
                self.has_open_paths = true;
                (*(*e_start).prev).out_idx = SKIP;
            }

            e = e_start;
            loop {
                init_edge2(&mut *e, poly_type);
                e = (*e).next;
                if is_flat && (*e).curr.y != (*e_start).curr.y {
                    is_flat = false;
                }
                if e == e_start {
                    break;
                }
            }

            if is_flat {
                if closed {
                    return Ok(false);
                }
                (*(*e).prev).out_idx = SKIP;
                let loc_min = LocalMinimum {
                    y: (*e).bot.y,
                    left_bound: ptr::null_mut(),
                    right_bound: e,
                };
                (*loc_min.right_bound).side = EdgeSide::Right;
                (*loc_min.right_bound).wind_delta = 0;
                loop {
                    if (*e).bot.x != (*(*e).prev).top.x {
                        reverse_horizontal(&mut *e);
                    }
                    if (*(*e).next).out_idx == SKIP {
                        break;
                    }
                    (*e).next_in_lml = (*e).next;
                    e = (*e).next;
                }
                self.minima_list.push(loc_min);
                self.edges.push(edges);
                return Ok(true);
            }

            self.edges.push(edges);
            let mut left_bound_is_forward;
            let mut e_min: *mut TEdge = ptr::null_mut();

            if (*(*e).prev).bot == (*(*e).prev).top {
                e = (*e).next;
            }

            loop {
                e = find_next_loc_min(e);
                if e == e_min {
                    break;
                } else if e_min.is_null() {
                    e_min = e;
                }

                let mut loc_min = LocalMinimum {
                    y: (*e).bot.y,
                    left_bound: ptr::null_mut(),
                    right_bound: ptr::null_mut(),
                };
                if (*e).dx < (*(*e).prev).dx {
                    loc_min.left_bound = (*e).prev;
                    loc_min.right_bound = e;
                    left_bound_is_forward = false;
                } else {
                    loc_min.left_bound = e;
                    loc_min.right_bound = (*e).prev;
                    left_bound_is_forward = true;
                }

                if !closed {
                    (*loc_min.left_bound).wind_delta = 0;
                } else if (*loc_min.left_bound).next == loc_min.right_bound {
                    (*loc_min.left_bound).wind_delta = -1;
                } else {
                    (*loc_min.left_bound).wind_delta = 1;
                }
                (*loc_min.right_bound).wind_delta = -(*loc_min.left_bound).wind_delta;

                e = self.process_bound(loc_min.left_bound, left_bound_is_forward);
                if (*e).out_idx == SKIP {
                    e = self.process_bound(e, left_bound_is_forward);
                }

                let mut e2 = self.process_bound(loc_min.right_bound, !left_bound_is_forward);
                if (*e2).out_idx == SKIP {
                    e2 = self.process_bound(e2, !left_bound_is_forward);
                }

                if (*loc_min.left_bound).out_idx == SKIP {
                    loc_min.left_bound = ptr::null_mut();
                } else if (*loc_min.right_bound).out_idx == SKIP {
                    loc_min.right_bound = ptr::null_mut();
                }
                self.minima_list.push(loc_min);
                if !left_bound_is_forward {
                    e = e2;
                }
            }
        }

        Ok(true)
    }

    // C++: ClipperBase::AddPaths
    pub fn add_paths(&mut self, ppg: &[Path], poly_type: PolyType, closed: bool) -> Result<bool> {
        let mut result = false;
        for pg in ppg {
            if self.add_path(pg, poly_type, closed)? {
                result = true;
            }
        }
        Ok(result)
    }

    // C++: ClipperBase::Clear
    pub unsafe fn clear(&mut self) {
        self.dispose_local_minima_list();
        unsafe {
            self.dispose_all_out_recs();
        }
        self.edges.clear();
        self.use_full_range = false;
        self.has_open_paths = false;
        self.active_edges = ptr::null_mut();
        self.scanbeam.clear();
    }

    // C++: ClipperBase::Reset
    pub unsafe fn reset(&mut self) {
        self.current_lm = 0;
        if self.current_lm == self.minima_list.len() {
            return;
        }

        self.minima_list
            .sort_by(|loc_min1, loc_min2| loc_min2.y.cmp(&loc_min1.y));
        self.scanbeam.clear();

        for i in 0..self.minima_list.len() {
            let y = self.minima_list[i].y;
            self.insert_scanbeam(y);

            let e = self.minima_list[i].left_bound;
            if !e.is_null() {
                // SAFETY: local minima bounds point into owned edge arrays.
                unsafe {
                    (*e).curr = (*e).bot;
                    (*e).side = EdgeSide::Left;
                    (*e).out_idx = UNASSIGNED;
                }
            }

            let e = self.minima_list[i].right_bound;
            if !e.is_null() {
                // SAFETY: local minima bounds point into owned edge arrays.
                unsafe {
                    (*e).curr = (*e).bot;
                    (*e).side = EdgeSide::Right;
                    (*e).out_idx = UNASSIGNED;
                }
            }
        }

        self.active_edges = ptr::null_mut();
        self.current_lm = 0;
    }

    // C++: ClipperBase::DisposeLocalMinimaList
    pub fn dispose_local_minima_list(&mut self) {
        self.minima_list.clear();
        self.current_lm = 0;
    }

    // C++: ClipperBase::PopLocalMinima
    pub fn pop_local_minima(&mut self, y: CInt) -> Option<LocalMinimum> {
        if self.current_lm == self.minima_list.len() || self.minima_list[self.current_lm].y != y {
            return None;
        }
        let loc_min = self.minima_list[self.current_lm];
        self.current_lm += 1;
        Some(loc_min)
    }

    // C++: ClipperBase::GetBounds
    pub unsafe fn get_bounds(&self) -> IntRect {
        let Some(first_lm) = self.minima_list.first() else {
            return IntRect::default();
        };
        if first_lm.left_bound.is_null() {
            return IntRect::default();
        }

        // SAFETY: local minima bounds point into owned edge arrays.
        unsafe {
            let first = first_lm.left_bound;
            let mut result = IntRect {
                left: (*first).bot.x,
                top: (*first).bot.y,
                right: (*first).bot.x,
                bottom: (*first).bot.y,
            };

            for lm in &self.minima_list {
                if lm.left_bound.is_null() {
                    continue;
                }

                result.bottom = result.bottom.max((*lm.left_bound).bot.y);
                let mut e = lm.left_bound;
                loop {
                    let bottom_e = e;
                    while !(*e).next_in_lml.is_null() {
                        result.left = result.left.min((*e).bot.x);
                        result.right = result.right.max((*e).bot.x);
                        e = (*e).next_in_lml;
                    }
                    result.left = result.left.min((*e).bot.x);
                    result.right = result.right.max((*e).bot.x);
                    result.left = result.left.min((*e).top.x);
                    result.right = result.right.max((*e).top.x);
                    result.top = result.top.min((*e).top.y);
                    if bottom_e == lm.left_bound {
                        e = lm.right_bound;
                        if e.is_null() {
                            break;
                        }
                    } else {
                        break;
                    }
                }
            }

            result
        }
    }

    // C++: ClipperBase::InsertScanbeam
    pub fn insert_scanbeam(&mut self, y: CInt) {
        self.scanbeam.push(y);
    }

    // C++: ClipperBase::PopScanbeam
    pub fn pop_scanbeam(&mut self) -> Option<CInt> {
        let y = self.scanbeam.pop()?;
        while self.scanbeam.peek() == Some(&y) {
            self.scanbeam.pop();
        }
        Some(y)
    }

    // C++: ClipperBase::DisposeAllOutRecs
    pub unsafe fn dispose_all_out_recs(&mut self) {
        for i in 0..self.poly_outs.len() {
            unsafe {
                self.dispose_out_rec(i);
            }
        }
        self.poly_outs.clear();
        self.out_rec_arena.clear();
        self.out_pt_arena.clear();
    }

    // C++: ClipperBase::DisposeOutRec
    pub unsafe fn dispose_out_rec(&mut self, index: usize) {
        let out_rec = self.poly_outs[index];
        if out_rec.is_null() {
            return;
        }

        // SAFETY: poly_outs entries point into out_rec_arena and remain valid
        // until dispose_all_out_recs clears the arena.
        unsafe {
            (*out_rec).pts = ptr::null_mut();
        }
        self.poly_outs[index] = ptr::null_mut();
    }

    // C++: ClipperBase::DeleteFromAEL
    pub unsafe fn delete_from_ael(&mut self, e: *mut TEdge) {
        // SAFETY: caller provides an edge that may be present in AEL.
        unsafe {
            let ael_prev = (*e).prev_in_ael;
            let ael_next = (*e).next_in_ael;
            if ael_prev.is_null() && ael_next.is_null() && e != self.active_edges {
                return;
            }
            if !ael_prev.is_null() {
                (*ael_prev).next_in_ael = ael_next;
            } else {
                self.active_edges = ael_next;
            }
            if !ael_next.is_null() {
                (*ael_next).prev_in_ael = ael_prev;
            }
            (*e).next_in_ael = ptr::null_mut();
            (*e).prev_in_ael = ptr::null_mut();
        }
    }

    // C++: ClipperBase::CreateOutRec
    pub fn create_out_rec(&mut self) -> *mut OutRec {
        self.out_rec_arena.push(Box::new(OutRec {
            idx: self.poly_outs.len() as i32,
            ..OutRec::default()
        }));
        let result = &mut **self.out_rec_arena.last_mut().unwrap() as *mut OutRec;
        self.poly_outs.push(result);
        result
    }

    pub fn create_out_pt(&mut self, out_pt: OutPt) -> *mut OutPt {
        self.out_pt_arena.alloc(out_pt)
    }

    // C++: ClipperBase::SwapPositionsInAEL
    pub unsafe fn swap_positions_in_ael(&mut self, edge1: *mut TEdge, edge2: *mut TEdge) {
        // SAFETY: caller provides valid AEL edge pointers.
        unsafe {
            if (*edge1).next_in_ael == (*edge1).prev_in_ael
                || (*edge2).next_in_ael == (*edge2).prev_in_ael
            {
                return;
            }

            if (*edge1).next_in_ael == edge2 {
                let next = (*edge2).next_in_ael;
                if !next.is_null() {
                    (*next).prev_in_ael = edge1;
                }
                let prev = (*edge1).prev_in_ael;
                if !prev.is_null() {
                    (*prev).next_in_ael = edge2;
                }
                (*edge2).prev_in_ael = prev;
                (*edge2).next_in_ael = edge1;
                (*edge1).prev_in_ael = edge2;
                (*edge1).next_in_ael = next;
            } else if (*edge2).next_in_ael == edge1 {
                let next = (*edge1).next_in_ael;
                if !next.is_null() {
                    (*next).prev_in_ael = edge2;
                }
                let prev = (*edge2).prev_in_ael;
                if !prev.is_null() {
                    (*prev).next_in_ael = edge1;
                }
                (*edge1).prev_in_ael = prev;
                (*edge1).next_in_ael = edge2;
                (*edge2).prev_in_ael = edge1;
                (*edge2).next_in_ael = next;
            } else {
                let next = (*edge1).next_in_ael;
                let prev = (*edge1).prev_in_ael;
                (*edge1).next_in_ael = (*edge2).next_in_ael;
                if !(*edge1).next_in_ael.is_null() {
                    (*(*edge1).next_in_ael).prev_in_ael = edge1;
                }
                (*edge1).prev_in_ael = (*edge2).prev_in_ael;
                if !(*edge1).prev_in_ael.is_null() {
                    (*(*edge1).prev_in_ael).next_in_ael = edge1;
                }
                (*edge2).next_in_ael = next;
                if !(*edge2).next_in_ael.is_null() {
                    (*(*edge2).next_in_ael).prev_in_ael = edge2;
                }
                (*edge2).prev_in_ael = prev;
                if !(*edge2).prev_in_ael.is_null() {
                    (*(*edge2).prev_in_ael).next_in_ael = edge2;
                }
            }

            if (*edge1).prev_in_ael.is_null() {
                self.active_edges = edge1;
            } else if (*edge2).prev_in_ael.is_null() {
                self.active_edges = edge2;
            }
        }
    }

    // C++: ClipperBase::UpdateEdgeIntoAEL
    pub unsafe fn update_edge_into_ael(&mut self, e: &mut *mut TEdge) -> Result<()> {
        // SAFETY: caller provides an active edge pointer.
        unsafe {
            if (**e).next_in_lml.is_null() {
                return Err(ClipperError::new("UpdateEdgeIntoAEL: invalid call"));
            }

            (*(**e).next_in_lml).out_idx = (**e).out_idx;
            let ael_prev = (**e).prev_in_ael;
            let ael_next = (**e).next_in_ael;
            if !ael_prev.is_null() {
                (*ael_prev).next_in_ael = (**e).next_in_lml;
            } else {
                self.active_edges = (**e).next_in_lml;
            }
            if !ael_next.is_null() {
                (*ael_next).prev_in_ael = (**e).next_in_lml;
            }
            (*(**e).next_in_lml).side = (**e).side;
            (*(**e).next_in_lml).wind_delta = (**e).wind_delta;
            (*(**e).next_in_lml).wind_cnt = (**e).wind_cnt;
            (*(**e).next_in_lml).wind_cnt2 = (**e).wind_cnt2;
            *e = (**e).next_in_lml;
            (**e).curr = (**e).bot;
            (**e).prev_in_ael = ael_prev;
            (**e).next_in_ael = ael_next;
            if !is_horizontal(&**e) {
                self.insert_scanbeam((**e).top.y);
            }
        }
        Ok(())
    }

    // C++: ClipperBase::LocalMinimaPending
    pub fn local_minima_pending(&self) -> bool {
        self.current_lm != self.minima_list.len()
    }

    // C++: ClipperBase::ProcessBound
    unsafe fn process_bound(&mut self, mut e: *mut TEdge, next_is_forward: bool) -> *mut TEdge {
        // SAFETY: caller provides an edge in a valid bound ring.
        unsafe {
            let mut result = e;
            let mut horz: *mut TEdge;

            if (*e).out_idx == SKIP {
                if next_is_forward {
                    while (*e).top.y == (*(*e).next).bot.y {
                        e = (*e).next;
                    }
                    while e != result && is_horizontal(&*e) {
                        e = (*e).prev;
                    }
                } else {
                    while (*e).top.y == (*(*e).prev).bot.y {
                        e = (*e).prev;
                    }
                    while e != result && is_horizontal(&*e) {
                        e = (*e).next;
                    }
                }

                if e == result {
                    if next_is_forward {
                        result = (*e).next;
                    } else {
                        result = (*e).prev;
                    }
                } else {
                    if next_is_forward {
                        e = (*result).next;
                    } else {
                        e = (*result).prev;
                    }
                    let loc_min = LocalMinimum {
                        y: (*e).bot.y,
                        left_bound: ptr::null_mut(),
                        right_bound: e,
                    };
                    (*e).wind_delta = 0;
                    result = self.process_bound(e, next_is_forward);
                    self.minima_list.push(loc_min);
                }
                return result;
            }

            let mut e_start;
            if is_horizontal(&*e) {
                if next_is_forward {
                    e_start = (*e).prev;
                } else {
                    e_start = (*e).next;
                }
                if is_horizontal(&*e_start) {
                    if (*e_start).bot.x != (*e).bot.x && (*e_start).top.x != (*e).bot.x {
                        reverse_horizontal(&mut *e);
                    }
                } else if (*e_start).bot.x != (*e).bot.x {
                    reverse_horizontal(&mut *e);
                }
            }

            e_start = e;
            if next_is_forward {
                while (*result).top.y == (*(*result).next).bot.y
                    && (*(*result).next).out_idx != SKIP
                {
                    result = (*result).next;
                }
                if is_horizontal(&*result) && (*(*result).next).out_idx != SKIP {
                    horz = result;
                    while is_horizontal(&*(*horz).prev) {
                        horz = (*horz).prev;
                    }
                    if (*(*horz).prev).top.x > (*(*result).next).top.x {
                        result = (*horz).prev;
                    }
                }
                while e != result {
                    (*e).next_in_lml = (*e).next;
                    if is_horizontal(&*e) && e != e_start && (*e).bot.x != (*(*e).prev).top.x {
                        reverse_horizontal(&mut *e);
                    }
                    e = (*e).next;
                }
                if is_horizontal(&*e) && e != e_start && (*e).bot.x != (*(*e).prev).top.x {
                    reverse_horizontal(&mut *e);
                }
                result = (*result).next;
            } else {
                while (*result).top.y == (*(*result).prev).bot.y
                    && (*(*result).prev).out_idx != SKIP
                {
                    result = (*result).prev;
                }
                if is_horizontal(&*result) && (*(*result).prev).out_idx != SKIP {
                    horz = result;
                    while is_horizontal(&*(*horz).next) {
                        horz = (*horz).next;
                    }
                    if (*(*horz).next).top.x == (*(*result).prev).top.x
                        || (*(*horz).next).top.x > (*(*result).prev).top.x
                    {
                        result = (*horz).next;
                    }
                }

                while e != result {
                    (*e).next_in_lml = (*e).prev;
                    if is_horizontal(&*e) && e != e_start && (*e).bot.x != (*(*e).next).top.x {
                        reverse_horizontal(&mut *e);
                    }
                    e = (*e).prev;
                }
                if is_horizontal(&*e) && e != e_start && (*e).bot.x != (*(*e).next).top.x {
                    reverse_horizontal(&mut *e);
                }
                result = (*result).prev;
            }

            result
        }
    }
}

// C++: FindNextLocMin
unsafe fn find_next_loc_min(mut e: *mut TEdge) -> *mut TEdge {
    // SAFETY: caller provides an edge in a valid ring.
    unsafe {
        loop {
            while (*e).bot != (*(*e).prev).bot || (*e).curr == (*e).top {
                e = (*e).next;
            }
            if !is_horizontal(&*e) && !is_horizontal(&*(*e).prev) {
                break;
            }
            while is_horizontal(&*(*e).prev) {
                e = (*e).prev;
            }
            let e2 = e;
            while is_horizontal(&*e) {
                e = (*e).next;
            }
            if (*e).top.y == (*(*e).prev).bot.y {
                continue;
            }
            if (*(*e2).prev).bot.x < (*e).bot.x {
                e = e2;
            }
            break;
        }
        e
    }
}

impl Drop for ClipperBase {
    fn drop(&mut self) {
        // SAFETY: mirrors ClipperBase::~ClipperBase.
        unsafe {
            self.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{IntPoint, SKIP};

    #[test]
    fn scanbeam_pops_largest_y_and_discards_duplicates() {
        let mut base = ClipperBase::new();
        base.insert_scanbeam(5);
        base.insert_scanbeam(10);
        base.insert_scanbeam(10);
        base.insert_scanbeam(1);

        assert_eq!(base.pop_scanbeam(), Some(10));
        assert_eq!(base.pop_scanbeam(), Some(5));
        assert_eq!(base.pop_scanbeam(), Some(1));
        assert_eq!(base.pop_scanbeam(), None);
    }

    #[test]
    fn create_and_dispose_out_rec_tracks_indices() {
        let mut base = ClipperBase::new();
        let out0 = base.create_out_rec();
        let out1 = base.create_out_rec();

        unsafe {
            assert_eq!((*out0).idx, 0);
            assert_eq!((*out1).idx, 1);
            base.dispose_out_rec(0);
        }

        assert!(base.poly_outs[0].is_null());
        assert!(!base.poly_outs[1].is_null());
    }

    #[test]
    fn delete_from_ael_unlinks_edge() {
        let mut base = ClipperBase::new();
        let mut edges = vec![TEdge::default(), TEdge::default(), TEdge::default()];
        let e0 = edges.as_mut_ptr();
        let e1 = unsafe { e0.add(1) };
        let e2 = unsafe { e0.add(2) };

        unsafe {
            (*e0).next_in_ael = e1;
            (*e1).prev_in_ael = e0;
            (*e1).next_in_ael = e2;
            (*e2).prev_in_ael = e1;
            base.active_edges = e0;

            base.delete_from_ael(e1);

            assert_eq!((*e0).next_in_ael, e2);
            assert_eq!((*e2).prev_in_ael, e0);
            assert!((*e1).next_in_ael.is_null());
            assert!((*e1).prev_in_ael.is_null());
        }
    }

    #[test]
    fn swap_positions_in_ael_handles_adjacent_edges() {
        let mut base = ClipperBase::new();
        let mut edges = vec![TEdge::default(), TEdge::default(), TEdge::default()];
        let e0 = edges.as_mut_ptr();
        let e1 = unsafe { e0.add(1) };
        let e2 = unsafe { e0.add(2) };

        unsafe {
            (*e0).next_in_ael = e1;
            (*e1).prev_in_ael = e0;
            (*e1).next_in_ael = e2;
            (*e2).prev_in_ael = e1;
            base.active_edges = e0;

            base.swap_positions_in_ael(e0, e1);

            assert_eq!(base.active_edges, e1);
            assert_eq!((*e1).next_in_ael, e0);
            assert_eq!((*e0).prev_in_ael, e1);
            assert_eq!((*e0).next_in_ael, e2);
            assert_eq!((*e2).prev_in_ael, e0);
        }
    }

    #[test]
    fn update_edge_into_ael_replaces_edge_with_next_in_lml() {
        let mut base = ClipperBase::new();
        let mut edges = vec![TEdge::default(), TEdge::default()];
        let e0 = edges.as_mut_ptr();
        let e1 = unsafe { e0.add(1) };

        unsafe {
            (*e0).out_idx = 7;
            (*e0).side = EdgeSide::Right;
            (*e0).wind_delta = -1;
            (*e0).wind_cnt = 2;
            (*e0).wind_cnt2 = 3;
            (*e0).next_in_lml = e1;
            (*e0).bot = IntPoint::new(0, 10);
            (*e0).curr = IntPoint::new(0, 10);
            (*e0).top = IntPoint::new(10, 0);

            (*e1).bot = IntPoint::new(10, 0);
            (*e1).top = IntPoint::new(20, -10);
            (*e1).dx = 1.0;
            base.active_edges = e0;

            let mut edge = e0;
            base.update_edge_into_ael(&mut edge).unwrap();

            assert_eq!(edge, e1);
            assert_eq!(base.active_edges, e1);
            assert_eq!((*e1).out_idx, 7);
            assert_eq!((*e1).side, EdgeSide::Right);
            assert_eq!((*e1).curr, (*e1).bot);
            assert_eq!(base.pop_scanbeam(), Some(-10));
        }
    }

    #[test]
    fn pop_local_minima_advances_current_index() {
        let mut base = ClipperBase::new();
        let mut edge = TEdge {
            out_idx: SKIP,
            ..TEdge::default()
        };
        let edge_ptr = &mut edge as *mut TEdge;
        base.minima_list.push(LocalMinimum {
            y: 4,
            left_bound: edge_ptr,
            right_bound: ptr::null_mut(),
        });

        assert!(base.local_minima_pending());
        assert!(base.pop_local_minima(5).is_none());
        assert_eq!(base.pop_local_minima(4).unwrap().left_bound, edge_ptr);
        assert!(!base.local_minima_pending());
    }

    #[test]
    fn add_path_rejects_open_clip_paths_with_use_lines_enabled() {
        let mut base = ClipperBase::new();
        let path = vec![IntPoint::new(0, 0), IntPoint::new(10, 0)];

        let err = base.add_path(&path, PolyType::Clip, false).unwrap_err();

        assert_eq!(err.to_string(), "AddPath: Open paths must be subject.");
    }

    #[test]
    fn add_path_accepts_open_subject_path() {
        let mut base = ClipperBase::new();
        let path = vec![IntPoint::new(0, 0), IntPoint::new(10, 0)];

        assert!(base.add_path(&path, PolyType::Subject, false).unwrap());
        assert!(base.has_open_paths);
        assert_eq!(base.edges.len(), 1);
        assert_eq!(base.minima_list.len(), 1);
        assert!(base.minima_list[0].left_bound.is_null());
        assert!(!base.minima_list[0].right_bound.is_null());
    }

    #[test]
    fn add_path_accepts_closed_polygon_and_computes_bounds() {
        let mut base = ClipperBase::new();
        let path = vec![
            IntPoint::new(0, 0),
            IntPoint::new(10, 0),
            IntPoint::new(10, 10),
            IntPoint::new(0, 10),
        ];

        assert!(base.add_path(&path, PolyType::Subject, true).unwrap());
        assert_eq!(base.edges.len(), 1);
        assert_eq!(base.minima_list.len(), 1);

        let bounds = unsafe { base.get_bounds() };
        assert_eq!(bounds.left, 0);
        assert_eq!(bounds.top, 0);
        assert_eq!(bounds.right, 10);
        assert_eq!(bounds.bottom, 10);

        unsafe {
            base.reset();
        }
        assert!(base.local_minima_pending());
        assert_eq!(base.pop_scanbeam(), Some(10));
    }
}
