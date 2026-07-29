use crate::clipper::Clipper;
use crate::error::{ClipperError, Result};
use crate::types::{
    CInt, ClipType, DoublePoint, HI_RANGE, HORIZONTAL, IntPoint, LO_RANGE, OutPt, Path, Paths,
    PolyFillType, PolyNode, PolyTree, PolyType, TEdge, TOLERANCE, UNASSIGNED,
};

pub fn near_zero(val: f64) -> bool {
    val > -TOLERANCE && val < TOLERANCE
}

// C++: Round
pub fn round(val: f64) -> CInt {
    if val < 0.0 {
        (val - 0.5) as CInt
    } else {
        (val + 0.5) as CInt
    }
}

// C++: Abs
pub fn abs(val: CInt) -> CInt {
    if val < 0 { -val } else { val }
}

// C++: Area(const Path&)
pub fn area(poly: &Path) -> f64 {
    let size = poly.len();
    if size < 3 {
        return 0.0;
    }

    let mut a = 0.0;
    let mut j = size - 1;
    for i in 0..size {
        a += ((poly[j].x as f64) + (poly[i].x as f64)) * ((poly[j].y as f64) - (poly[i].y as f64));
        j = i;
    }
    -a * 0.5
}

// C++: Area(const OutPt*)
pub unsafe fn area_out_pt(op: *const OutPt) -> f64 {
    if op.is_null() {
        return 0.0;
    }

    let start_op = op;
    let mut op = op;
    let mut a = 0.0;
    loop {
        // SAFETY: caller provides a valid circular OutPt list.
        unsafe {
            a += (((*(*op).prev).pt.x + (*op).pt.x) as f64)
                * (((*(*op).prev).pt.y - (*op).pt.y) as f64);
            op = (*op).next;
        }
        if op == start_op {
            break;
        }
    }
    a * 0.5
}

// C++: PointIsVertex
pub unsafe fn point_is_vertex(pt: IntPoint, pp: *mut OutPt) -> bool {
    unsafe {
        let mut pp2 = pp;
        loop {
            if (*pp2).pt == pt {
                return true;
            }
            pp2 = (*pp2).next;
            if pp2 == pp {
                break;
            }
        }
    }
    false
}

// C++: Orientation
pub fn orientation(poly: &Path) -> bool {
    area(poly) >= 0.0
}

// C++: PointInPolygon(const IntPoint&, const Path&)
pub fn point_in_polygon(pt: IntPoint, path: &Path) -> i32 {
    let mut result = 0;
    let cnt = path.len();
    if cnt < 3 {
        return 0;
    }

    let mut ip = path[0];
    for i in 1..=cnt {
        let ip_next = if i == cnt { path[0] } else { path[i] };
        if ip_next.y == pt.y
            && (ip_next.x == pt.x || (ip.y == pt.y && ((ip_next.x > pt.x) == (ip.x < pt.x))))
        {
            return -1;
        }
        if (ip.y < pt.y) != (ip_next.y < pt.y) {
            if ip.x >= pt.x {
                if ip_next.x > pt.x {
                    result = 1 - result;
                } else {
                    let d = ((ip.x - pt.x) as f64) * ((ip_next.y - pt.y) as f64)
                        - ((ip_next.x - pt.x) as f64) * ((ip.y - pt.y) as f64);
                    if d == 0.0 {
                        return -1;
                    }
                    if (d > 0.0) == (ip_next.y > ip.y) {
                        result = 1 - result;
                    }
                }
            } else if ip_next.x > pt.x {
                let d = ((ip.x - pt.x) as f64) * ((ip_next.y - pt.y) as f64)
                    - ((ip_next.x - pt.x) as f64) * ((ip.y - pt.y) as f64);
                if d == 0.0 {
                    return -1;
                }
                if (d > 0.0) == (ip_next.y > ip.y) {
                    result = 1 - result;
                }
            }
        }
        ip = ip_next;
    }
    result
}

// C++: PointInPolygon(const IntPoint&, OutPt*)
pub unsafe fn point_in_polygon_out_pt(pt: IntPoint, op: *mut OutPt) -> i32 {
    let mut result = 0;
    let start_op = op;
    let mut op = op;
    unsafe {
        loop {
            if (*(*op).next).pt.y == pt.y
                && ((*(*op).next).pt.x == pt.x
                    || ((*op).pt.y == pt.y && (((*(*op).next).pt.x > pt.x) == ((*op).pt.x < pt.x))))
            {
                return -1;
            }
            if ((*op).pt.y < pt.y) != ((*(*op).next).pt.y < pt.y) {
                if (*op).pt.x >= pt.x {
                    if (*(*op).next).pt.x > pt.x {
                        result = 1 - result;
                    } else {
                        let d = (((*op).pt.x - pt.x) as f64) * (((*(*op).next).pt.y - pt.y) as f64)
                            - (((*(*op).next).pt.x - pt.x) as f64) * (((*op).pt.y - pt.y) as f64);
                        if d == 0.0 {
                            return -1;
                        }
                        if (d > 0.0) == ((*(*op).next).pt.y > (*op).pt.y) {
                            result = 1 - result;
                        }
                    }
                } else if (*(*op).next).pt.x > pt.x {
                    let d = (((*op).pt.x - pt.x) as f64) * (((*(*op).next).pt.y - pt.y) as f64)
                        - (((*(*op).next).pt.x - pt.x) as f64) * (((*op).pt.y - pt.y) as f64);
                    if d == 0.0 {
                        return -1;
                    }
                    if (d > 0.0) == ((*(*op).next).pt.y > (*op).pt.y) {
                        result = 1 - result;
                    }
                }
            }
            op = (*op).next;
            if start_op == op {
                break;
            }
        }
    }
    result
}

// C++: Poly2ContainsPoly1
pub unsafe fn poly2_contains_poly1(out_pt1: *mut OutPt, out_pt2: *mut OutPt) -> bool {
    unsafe {
        let mut op = out_pt1;
        loop {
            let res = point_in_polygon_out_pt((*op).pt, out_pt2);
            if res >= 0 {
                return res > 0;
            }
            op = (*op).next;
            if op == out_pt1 {
                break;
            }
        }
    }
    true
}

// C++: SlopesEqual(const TEdge&, const TEdge&, bool)
pub fn slopes_equal_edges(e1: &TEdge, e2: &TEdge, use_full_int64_range: bool) -> bool {
    if use_full_int64_range {
        int128_mul(e1.top.y - e1.bot.y, e2.top.x - e2.bot.x)
            == int128_mul(e1.top.x - e1.bot.x, e2.top.y - e2.bot.y)
    } else {
        (e1.top.y - e1.bot.y) * (e2.top.x - e2.bot.x)
            == (e1.top.x - e1.bot.x) * (e2.top.y - e2.bot.y)
    }
}

// C++: SlopesEqual(const IntPoint, const IntPoint, const IntPoint, bool)
pub fn slopes_equal_3_points(
    pt1: IntPoint,
    pt2: IntPoint,
    pt3: IntPoint,
    use_full_int64_range: bool,
) -> bool {
    if use_full_int64_range {
        int128_mul(pt1.y - pt2.y, pt2.x - pt3.x) == int128_mul(pt1.x - pt2.x, pt2.y - pt3.y)
    } else {
        (pt1.y - pt2.y) * (pt2.x - pt3.x) == (pt1.x - pt2.x) * (pt2.y - pt3.y)
    }
}

// C++: SlopesEqual(const IntPoint, const IntPoint, const IntPoint, const IntPoint, bool)
pub fn slopes_equal_4_points(
    pt1: IntPoint,
    pt2: IntPoint,
    pt3: IntPoint,
    pt4: IntPoint,
    use_full_int64_range: bool,
) -> bool {
    if use_full_int64_range {
        int128_mul(pt1.y - pt2.y, pt3.x - pt4.x) == int128_mul(pt1.x - pt2.x, pt3.y - pt4.y)
    } else {
        (pt1.y - pt2.y) * (pt3.x - pt4.x) == (pt1.x - pt2.x) * (pt3.y - pt4.y)
    }
}

pub fn int128_mul(lhs: CInt, rhs: CInt) -> i128 {
    (lhs as i128) * (rhs as i128)
}

// C++: IsHorizontal
pub fn is_horizontal(e: &TEdge) -> bool {
    e.dx == HORIZONTAL
}

// C++: GetDx
pub fn get_dx(pt1: IntPoint, pt2: IntPoint) -> f64 {
    if pt1.y == pt2.y {
        HORIZONTAL
    } else {
        ((pt2.x - pt1.x) as f64) / ((pt2.y - pt1.y) as f64)
    }
}

// C++: SetDx
pub fn set_dx(e: &mut TEdge) {
    let dy = e.top.y - e.bot.y;
    if dy == 0 {
        e.dx = HORIZONTAL;
    } else {
        e.dx = ((e.top.x - e.bot.x) as f64) / (dy as f64);
    }
}

// C++: SwapSides
pub fn swap_sides(edge1: &mut TEdge, edge2: &mut TEdge) {
    let side = edge1.side;
    edge1.side = edge2.side;
    edge2.side = side;
}

// C++: SwapPolyIndexes
pub fn swap_poly_indexes(edge1: &mut TEdge, edge2: &mut TEdge) {
    let out_idx = edge1.out_idx;
    edge1.out_idx = edge2.out_idx;
    edge2.out_idx = out_idx;
}

// C++: TopX
pub fn top_x(edge: &TEdge, current_y: CInt) -> CInt {
    if current_y == edge.top.y {
        edge.top.x
    } else {
        edge.bot.x + round(edge.dx * ((current_y - edge.bot.y) as f64))
    }
}

// C++: IntersectPoint
pub fn intersect_point(edge1: &TEdge, edge2: &TEdge, ip: &mut IntPoint) {
    let (b1, b2);
    if edge1.dx == edge2.dx {
        ip.y = edge1.curr.y;
        ip.x = top_x(edge1, ip.y);
        return;
    } else if edge1.dx == 0.0 {
        ip.x = edge1.bot.x;
        if is_horizontal(edge2) {
            ip.y = edge2.bot.y;
        } else {
            b2 = (edge2.bot.y as f64) - ((edge2.bot.x as f64) / edge2.dx);
            ip.y = round((ip.x as f64) / edge2.dx + b2);
        }
    } else if edge2.dx == 0.0 {
        ip.x = edge2.bot.x;
        if is_horizontal(edge1) {
            ip.y = edge1.bot.y;
        } else {
            b1 = (edge1.bot.y as f64) - ((edge1.bot.x as f64) / edge1.dx);
            ip.y = round((ip.x as f64) / edge1.dx + b1);
        }
    } else {
        b1 = (edge1.bot.x as f64) - (edge1.bot.y as f64) * edge1.dx;
        b2 = (edge2.bot.x as f64) - (edge2.bot.y as f64) * edge2.dx;
        let q = (b2 - b1) / (edge1.dx - edge2.dx);
        ip.y = round(q);
        if edge1.dx.abs() < edge2.dx.abs() {
            ip.x = round(edge1.dx * q + b1);
        } else {
            ip.x = round(edge2.dx * q + b2);
        }
    }

    if ip.y < edge1.top.y || ip.y < edge2.top.y {
        if edge1.top.y > edge2.top.y {
            ip.y = edge1.top.y;
        } else {
            ip.y = edge2.top.y;
        }
        if edge1.dx.abs() < edge2.dx.abs() {
            ip.x = top_x(edge1, ip.y);
        } else {
            ip.x = top_x(edge2, ip.y);
        }
    }

    if ip.y > edge1.curr.y {
        ip.y = edge1.curr.y;
        if edge1.dx.abs() > edge2.dx.abs() {
            ip.x = top_x(edge2, ip.y);
        } else {
            ip.x = top_x(edge1, ip.y);
        }
    }
}

// C++: ReversePolyPtLinks
pub unsafe fn reverse_poly_pt_links(pp: *mut OutPt) {
    if pp.is_null() {
        return;
    }

    let mut pp1 = pp;
    loop {
        // SAFETY: caller provides a valid circular OutPt list.
        unsafe {
            let pp2 = (*pp1).next;
            (*pp1).next = (*pp1).prev;
            (*pp1).prev = pp2;
            pp1 = pp2;
        }
        if pp1 == pp {
            break;
        }
    }
}

// C++: DisposeOutPts
pub unsafe fn dispose_out_pts(pp: &mut *mut OutPt) {
    if pp.is_null() {
        return;
    }

    // SAFETY: caller provides a valid circular OutPt list allocated with Box::into_raw.
    unsafe {
        (*(**pp).prev).next = std::ptr::null_mut();
    }
    while !pp.is_null() {
        // SAFETY: list is broken above, so following next walks each allocation once.
        unsafe {
            let tmp_pp = *pp;
            *pp = (**pp).next;
            drop(Box::from_raw(tmp_pp));
        }
    }
}

// C++: InitEdge
pub unsafe fn init_edge(e: *mut TEdge, e_next: *mut TEdge, e_prev: *mut TEdge, pt: IntPoint) {
    // SAFETY: caller provides a valid TEdge pointer.
    unsafe {
        *e = TEdge::default();
        (*e).next = e_next;
        (*e).prev = e_prev;
        (*e).curr = pt;
        (*e).out_idx = UNASSIGNED;
    }
}

// C++: InitEdge2
pub unsafe fn init_edge2(e: &mut TEdge, poly_type: PolyType) {
    // SAFETY: caller initialized e.next to a valid edge, matching C++ preconditions.
    unsafe {
        if e.curr.y >= (*e.next).curr.y {
            e.bot = e.curr;
            e.top = (*e.next).curr;
        } else {
            e.top = e.curr;
            e.bot = (*e.next).curr;
        }
    }
    set_dx(e);
    e.poly_typ = poly_type;
}

// C++: RemoveEdge
pub unsafe fn remove_edge(e: *mut TEdge) -> *mut TEdge {
    // SAFETY: caller provides an edge in a valid doubly-linked ring.
    unsafe {
        (*(*e).prev).next = (*e).next;
        (*(*e).next).prev = (*e).prev;
        let result = (*e).next;
        (*e).prev = std::ptr::null_mut();
        result
    }
}

// C++: ReverseHorizontal
pub fn reverse_horizontal(e: &mut TEdge) {
    std::mem::swap(&mut e.top.x, &mut e.bot.x);
}

// C++: SwapPoints
pub fn swap_points(pt1: &mut IntPoint, pt2: &mut IntPoint) {
    std::mem::swap(pt1, pt2);
}

// C++: GetOverlapSegment
pub fn get_overlap_segment(
    mut pt1a: IntPoint,
    mut pt1b: IntPoint,
    mut pt2a: IntPoint,
    mut pt2b: IntPoint,
    pt1: &mut IntPoint,
    pt2: &mut IntPoint,
) -> bool {
    if abs(pt1a.x - pt1b.x) > abs(pt1a.y - pt1b.y) {
        if pt1a.x > pt1b.x {
            swap_points(&mut pt1a, &mut pt1b);
        }
        if pt2a.x > pt2b.x {
            swap_points(&mut pt2a, &mut pt2b);
        }
        if pt1a.x > pt2a.x {
            *pt1 = pt1a;
        } else {
            *pt1 = pt2a;
        }
        if pt1b.x < pt2b.x {
            *pt2 = pt1b;
        } else {
            *pt2 = pt2b;
        }
        pt1.x < pt2.x
    } else {
        if pt1a.y < pt1b.y {
            swap_points(&mut pt1a, &mut pt1b);
        }
        if pt2a.y < pt2b.y {
            swap_points(&mut pt2a, &mut pt2b);
        }
        if pt1a.y < pt2a.y {
            *pt1 = pt1a;
        } else {
            *pt1 = pt2a;
        }
        if pt1b.y > pt2b.y {
            *pt2 = pt1b;
        } else {
            *pt2 = pt2b;
        }
        pt1.y > pt2.y
    }
}

// C++: FirstIsBottomPt
pub unsafe fn first_is_bottom_pt(btm_pt1: *const OutPt, btm_pt2: *const OutPt) -> bool {
    // SAFETY: caller provides valid circular OutPt lists.
    unsafe {
        let mut p = (*btm_pt1).prev;
        while (*p).pt == (*btm_pt1).pt && !std::ptr::eq(p, btm_pt1.cast_mut()) {
            p = (*p).prev;
        }
        let dx1p = get_dx((*btm_pt1).pt, (*p).pt).abs();
        p = (*btm_pt1).next;
        while (*p).pt == (*btm_pt1).pt && !std::ptr::eq(p, btm_pt1.cast_mut()) {
            p = (*p).next;
        }
        let dx1n = get_dx((*btm_pt1).pt, (*p).pt).abs();

        p = (*btm_pt2).prev;
        while (*p).pt == (*btm_pt2).pt && !std::ptr::eq(p, btm_pt2.cast_mut()) {
            p = (*p).prev;
        }
        let dx2p = get_dx((*btm_pt2).pt, (*p).pt).abs();
        p = (*btm_pt2).next;
        while (*p).pt == (*btm_pt2).pt && !std::ptr::eq(p, btm_pt2.cast_mut()) {
            p = (*p).next;
        }
        let dx2n = get_dx((*btm_pt2).pt, (*p).pt).abs();

        if dx1p.max(dx1n) == dx2p.max(dx2n) && dx1p.min(dx1n) == dx2p.min(dx2n) {
            area_out_pt(btm_pt1) > 0.0
        } else {
            (dx1p >= dx2p && dx1p >= dx2n) || (dx1n >= dx2p && dx1n >= dx2n)
        }
    }
}

// C++: GetBottomPt
pub unsafe fn get_bottom_pt(mut pp: *mut OutPt) -> *mut OutPt {
    let mut dups: *mut OutPt = std::ptr::null_mut();
    // SAFETY: caller provides a valid circular OutPt list.
    unsafe {
        let mut p = (*pp).next;
        while p != pp {
            if (*p).pt.y > (*pp).pt.y {
                pp = p;
                dups = std::ptr::null_mut();
            } else if (*p).pt.y == (*pp).pt.y && (*p).pt.x <= (*pp).pt.x {
                if (*p).pt.x < (*pp).pt.x {
                    dups = std::ptr::null_mut();
                    pp = p;
                } else if (*p).next != pp && (*p).prev != pp {
                    dups = p;
                }
            }
            p = (*p).next;
        }
        if !dups.is_null() {
            while dups != p {
                if !first_is_bottom_pt(p, dups) {
                    pp = dups;
                }
                dups = (*dups).next;
                while (*dups).pt != (*pp).pt {
                    dups = (*dups).next;
                }
            }
        }
    }
    pp
}

// C++: Pt2IsBetweenPt1AndPt3
pub fn pt2_is_between_pt1_and_pt3(pt1: IntPoint, pt2: IntPoint, pt3: IntPoint) -> bool {
    if pt1 == pt3 || pt1 == pt2 || pt3 == pt2 {
        false
    } else if pt1.x != pt3.x {
        (pt2.x > pt1.x) == (pt2.x < pt3.x)
    } else {
        (pt2.y > pt1.y) == (pt2.y < pt3.y)
    }
}

// C++: HorzSegmentsOverlap
pub fn horz_segments_overlap(
    mut seg1a: CInt,
    mut seg1b: CInt,
    mut seg2a: CInt,
    mut seg2b: CInt,
) -> bool {
    if seg1a > seg1b {
        std::mem::swap(&mut seg1a, &mut seg1b);
    }
    if seg2a > seg2b {
        std::mem::swap(&mut seg2a, &mut seg2b);
    }
    (seg1a < seg2b) && (seg2a < seg1b)
}

// C++: E2InsertsBeforeE1
pub fn e2_inserts_before_e1(e1: &TEdge, e2: &TEdge) -> bool {
    if e2.curr.x == e1.curr.x {
        if e2.top.y > e1.top.y {
            e2.top.x < top_x(e1, e2.top.y)
        } else {
            e1.top.x > top_x(e2, e1.top.y)
        }
    } else {
        e2.curr.x < e1.curr.x
    }
}

// C++: GetOverlap
pub fn get_overlap(a1: CInt, a2: CInt, b1: CInt, b2: CInt) -> Option<(CInt, CInt)> {
    let (left, right) = if a1 < a2 {
        if b1 < b2 {
            (a1.max(b1), a2.min(b2))
        } else {
            (a1.max(b2), a2.min(b1))
        }
    } else if b1 < b2 {
        (a2.max(b1), a1.min(b2))
    } else {
        (a2.max(b2), a1.min(b1))
    };

    if left < right {
        Some((left, right))
    } else {
        None
    }
}

// C++: RangeTest
pub fn range_test(pt: IntPoint, use_full_range: &mut bool) -> Result<()> {
    if *use_full_range {
        if pt.x > HI_RANGE || pt.y > HI_RANGE || pt.x < -HI_RANGE || pt.y < -HI_RANGE {
            return Err(ClipperError::new("Coordinate outside allowed range"));
        }
    } else if pt.x > LO_RANGE || pt.y > LO_RANGE || pt.x < -LO_RANGE || pt.y < -LO_RANGE {
        *use_full_range = true;
        range_test(pt, use_full_range)?;
    }
    Ok(())
}

// C++: GetUnitNormal
pub fn get_unit_normal(pt1: IntPoint, pt2: IntPoint) -> DoublePoint {
    if pt2.x == pt1.x && pt2.y == pt1.y {
        return DoublePoint::new(0.0, 0.0);
    }

    let mut dx = (pt2.x - pt1.x) as f64;
    let mut dy = (pt2.y - pt1.y) as f64;
    let f = 1.0 / (dx * dx + dy * dy).sqrt();
    dx *= f;
    dy *= f;
    DoublePoint::new(dy, -dx)
}

// C++: ReversePath
pub fn reverse_path(p: &mut Path) {
    p.reverse();
}

// C++: ReversePaths
pub fn reverse_paths(p: &mut Paths) {
    for path in p {
        reverse_path(path);
    }
}

// C++: SimplifyPolygon
pub fn simplify_polygon_into(
    in_poly: &Path,
    out_polys: &mut Paths,
    fill_type: PolyFillType,
) -> Result<()> {
    let mut c = Clipper::new();
    c.set_strictly_simple(true);
    c.add_path(in_poly, PolyType::Subject, true)?;
    c.execute_with_fill_types(ClipType::Union, out_polys, fill_type, fill_type)?;
    Ok(())
}

pub fn simplify_polygon(in_poly: &Path, fill_type: PolyFillType) -> Result<Paths> {
    let mut out_polys = Vec::new();
    simplify_polygon_into(in_poly, &mut out_polys, fill_type)?;
    Ok(out_polys)
}

// C++: SimplifyPolygons(const Paths&, Paths&, PolyFillType)
pub fn simplify_polygons_into(
    in_polys: &Paths,
    out_polys: &mut Paths,
    fill_type: PolyFillType,
) -> Result<()> {
    let mut c = Clipper::new();
    c.set_strictly_simple(true);
    c.add_paths(in_polys, PolyType::Subject, true)?;
    c.execute_with_fill_types(ClipType::Union, out_polys, fill_type, fill_type)?;
    Ok(())
}

// C++: SimplifyPolygons(Paths&, PolyFillType)
pub fn simplify_polygons_mut(polys: &mut Paths, fill_type: PolyFillType) -> Result<()> {
    let input = polys.clone();
    simplify_polygons_into(&input, polys, fill_type)
}

// C++: DistanceSqrd
pub fn distance_sqrd(pt1: IntPoint, pt2: IntPoint) -> f64 {
    let dx = (pt1.x as f64) - (pt2.x as f64);
    let dy = (pt1.y as f64) - (pt2.y as f64);
    dx * dx + dy * dy
}

// C++: DistanceFromLineSqrd
pub fn distance_from_line_sqrd(pt: IntPoint, ln1: IntPoint, ln2: IntPoint) -> f64 {
    let a = (ln1.y - ln2.y) as f64;
    let b = (ln2.x - ln1.x) as f64;
    let c = a * (ln1.x as f64) + b * (ln1.y as f64);
    let c = a * (pt.x as f64) + b * (pt.y as f64) - c;
    (c * c) / (a * a + b * b)
}

// C++: SlopesNearCollinear
pub fn slopes_near_collinear(pt1: IntPoint, pt2: IntPoint, pt3: IntPoint, dist_sqrd: f64) -> bool {
    if abs(pt1.x - pt2.x) > abs(pt1.y - pt2.y) {
        if (pt1.x > pt2.x) == (pt1.x < pt3.x) {
            distance_from_line_sqrd(pt1, pt2, pt3) < dist_sqrd
        } else if (pt2.x > pt1.x) == (pt2.x < pt3.x) {
            distance_from_line_sqrd(pt2, pt1, pt3) < dist_sqrd
        } else {
            distance_from_line_sqrd(pt3, pt1, pt2) < dist_sqrd
        }
    } else if (pt1.y > pt2.y) == (pt1.y < pt3.y) {
        distance_from_line_sqrd(pt1, pt2, pt3) < dist_sqrd
    } else if (pt2.y > pt1.y) == (pt2.y < pt3.y) {
        distance_from_line_sqrd(pt2, pt1, pt3) < dist_sqrd
    } else {
        distance_from_line_sqrd(pt3, pt1, pt2) < dist_sqrd
    }
}

// C++: PointsAreClose
pub fn points_are_close(pt1: IntPoint, pt2: IntPoint, dist_sqrd: f64) -> bool {
    let dx = (pt1.x as f64) - (pt2.x as f64);
    let dy = (pt1.y as f64) - (pt2.y as f64);
    (dx * dx) + (dy * dy) <= dist_sqrd
}

// C++: ExcludeOp
unsafe fn exclude_op(op: *mut OutPt) -> *mut OutPt {
    // SAFETY: caller provides a valid node in a doubly-linked OutPt ring.
    unsafe {
        let result = (*op).prev;
        (*result).next = (*op).next;
        (*(*op).next).prev = result;
        (*result).idx = 0;
        result
    }
}

// C++: CleanPolygon(const Path&, Path&, double)
pub fn clean_polygon_into(in_poly: &Path, out_poly: &mut Path, distance: f64) {
    let mut size = in_poly.len();
    if size == 0 {
        out_poly.clear();
        return;
    }

    let mut out_pts: Vec<OutPt> = in_poly
        .iter()
        .map(|pt| OutPt {
            idx: 0,
            pt: *pt,
            next: std::ptr::null_mut(),
            prev: std::ptr::null_mut(),
        })
        .collect();

    let base = out_pts.as_mut_ptr();
    for i in 0..size {
        let next = (i + 1) % size;
        // SAFETY: base points to a Vec allocation with size initialized elements.
        unsafe {
            (*base.add(i)).next = base.add(next);
            (*base.add(next)).prev = base.add(i);
        }
    }

    let dist_sqrd = distance * distance;
    let mut op = base;
    unsafe {
        while (*op).idx == 0 && (*op).next != (*op).prev {
            if points_are_close((*op).pt, (*(*op).prev).pt, dist_sqrd) {
                op = exclude_op(op);
                size -= 1;
            } else if points_are_close((*(*op).prev).pt, (*(*op).next).pt, dist_sqrd) {
                exclude_op((*op).next);
                op = exclude_op(op);
                size -= 2;
            } else if slopes_near_collinear((*(*op).prev).pt, (*op).pt, (*(*op).next).pt, dist_sqrd)
            {
                op = exclude_op(op);
                size -= 1;
            } else {
                (*op).idx = 1;
                op = (*op).next;
            }
        }

        if size < 3 {
            size = 0;
        }
        out_poly.clear();
        out_poly.reserve(size);
        for _ in 0..size {
            out_poly.push((*op).pt);
            op = (*op).next;
        }
    }
}

// C++: CleanPolygon(Path&, double)
pub fn clean_polygon_mut(poly: &mut Path, distance: f64) {
    let input = poly.clone();
    clean_polygon_into(&input, poly, distance);
}

pub fn clean_polygon(in_poly: &Path, distance: f64) -> Path {
    let mut out_poly = Vec::new();
    clean_polygon_into(in_poly, &mut out_poly, distance);
    out_poly
}

// C++: CleanPolygons(const Paths&, Paths&, double)
pub fn clean_polygons_into(in_polys: &Paths, out_polys: &mut Paths, distance: f64) {
    out_polys.clear();
    out_polys.reserve(in_polys.len());
    for in_poly in in_polys {
        out_polys.push(clean_polygon(in_poly, distance));
    }
}

// C++: CleanPolygons(Paths&, double)
pub fn clean_polygons_mut(polys: &mut Paths, distance: f64) {
    let input = polys.clone();
    clean_polygons_into(&input, polys, distance);
}

pub fn clean_polygons(in_polys: &Paths, distance: f64) -> Paths {
    let mut out_polys = Vec::new();
    clean_polygons_into(in_polys, &mut out_polys, distance);
    out_polys
}

// C++: Minkowski
pub fn minkowski(poly: &Path, path: &Path, solution: &mut Paths, is_sum: bool, is_closed: bool) {
    let delta = if is_closed { 1 } else { 0 };
    let poly_cnt = poly.len();
    let path_cnt = path.len();

    let mut pp = Vec::with_capacity(path_cnt);
    if is_sum {
        for path_pt in path {
            let mut p = Vec::with_capacity(poly_cnt);
            for poly_pt in poly {
                p.push(IntPoint::new(path_pt.x + poly_pt.x, path_pt.y + poly_pt.y));
            }
            pp.push(p);
        }
    } else {
        for path_pt in path {
            let mut p = Vec::with_capacity(poly_cnt);
            for poly_pt in poly {
                p.push(IntPoint::new(path_pt.x - poly_pt.x, path_pt.y - poly_pt.y));
            }
            pp.push(p);
        }
    }

    solution.clear();
    if poly_cnt == 0 || path_cnt == 0 {
        return;
    }
    solution.reserve((path_cnt + delta) * (poly_cnt + 1));
    for i in 0..(path_cnt - 1 + delta) {
        for j in 0..poly_cnt {
            let mut quad = Vec::with_capacity(4);
            quad.push(pp[i % path_cnt][j % poly_cnt]);
            quad.push(pp[(i + 1) % path_cnt][j % poly_cnt]);
            quad.push(pp[(i + 1) % path_cnt][(j + 1) % poly_cnt]);
            quad.push(pp[i % path_cnt][(j + 1) % poly_cnt]);
            if !orientation(&quad) {
                reverse_path(&mut quad);
            }
            solution.push(quad);
        }
    }
}

// C++: MinkowskiSum(const Path&, const Path&, Paths&, bool)
pub fn minkowski_sum_into(
    pattern: &Path,
    path: &Path,
    solution: &mut Paths,
    path_is_closed: bool,
) -> Result<()> {
    minkowski(pattern, path, solution, true, path_is_closed);
    let mut c = Clipper::new();
    c.add_paths(solution, PolyType::Subject, true)?;
    c.execute_with_fill_types(
        ClipType::Union,
        solution,
        PolyFillType::NonZero,
        PolyFillType::NonZero,
    )?;
    Ok(())
}

// C++: MinkowskiSum(const Path&, const Paths&, Paths&, bool)
pub fn minkowski_sum_paths_into(
    pattern: &Path,
    paths: &Paths,
    solution: &mut Paths,
    path_is_closed: bool,
) -> Result<()> {
    let mut c = Clipper::new();
    for path in paths {
        let mut tmp = Vec::new();
        minkowski(pattern, path, &mut tmp, true, path_is_closed);
        c.add_paths(&tmp, PolyType::Subject, true)?;
        if path_is_closed {
            let mut tmp2 = Vec::new();
            translate_path(path, &mut tmp2, pattern[0]);
            c.add_path(&tmp2, PolyType::Clip, true)?;
        }
    }
    c.execute_with_fill_types(
        ClipType::Union,
        solution,
        PolyFillType::NonZero,
        PolyFillType::NonZero,
    )?;
    Ok(())
}

// C++: MinkowskiDiff
pub fn minkowski_diff_into(poly1: &Path, poly2: &Path, solution: &mut Paths) -> Result<()> {
    minkowski(poly1, poly2, solution, false, true);
    let mut c = Clipper::new();
    c.add_paths(solution, PolyType::Subject, true)?;
    c.execute_with_fill_types(
        ClipType::Union,
        solution,
        PolyFillType::NonZero,
        PolyFillType::NonZero,
    )?;
    Ok(())
}

// C++: TranslatePath
pub fn translate_path(input: &Path, output: &mut Path, delta: IntPoint) {
    output.clear();
    output.reserve(input.len());
    for pt in input {
        output.push(IntPoint::new(pt.x + delta.x, pt.y + delta.y));
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum NodeType {
    Any,
    Open,
    Closed,
}

// C++: AddPolyNodeToPaths
fn add_poly_node_to_paths(polynode: &PolyNode, nodetype: NodeType, paths: &mut Paths) {
    let mut is_match = true;
    if nodetype == NodeType::Closed {
        is_match = !polynode.is_open();
    } else if nodetype == NodeType::Open {
        return;
    }

    if !polynode.contour.is_empty() && is_match {
        paths.push(polynode.contour.clone());
    }
    for child in &polynode.childs {
        unsafe {
            add_poly_node_to_paths(&**child, nodetype, paths);
        }
    }
}

// C++: PolyTreeToPaths
pub fn poly_tree_to_paths(polytree: &PolyTree, paths: &mut Paths) {
    paths.clear();
    paths.reserve(polytree.total());
    add_poly_node_to_paths(&polytree.node, NodeType::Any, paths);
}

// C++: ClosedPathsFromPolyTree
pub fn closed_paths_from_poly_tree(polytree: &PolyTree, paths: &mut Paths) {
    paths.clear();
    paths.reserve(polytree.total());
    add_poly_node_to_paths(&polytree.node, NodeType::Closed, paths);
}

// C++: OpenPathsFromPolyTree
pub fn open_paths_from_poly_tree(polytree: &PolyTree, paths: &mut Paths) {
    paths.clear();
    paths.reserve(polytree.total());
    for child in &polytree.node.childs {
        unsafe {
            if (**child).is_open() {
                paths.push((**child).contour.clone());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{PolyNode, PolyTree, TEdge};

    #[test]
    fn round_matches_clipper_truncation() {
        assert_eq!(round(0.49), 0);
        assert_eq!(round(0.5), 1);
        assert_eq!(round(1.5), 2);
        assert_eq!(round(-0.49), 0);
        assert_eq!(round(-0.5), -1);
        assert_eq!(round(-1.5), -2);
    }

    #[test]
    fn area_and_orientation_match_path_direction() {
        let ccw = vec![
            IntPoint::new(0, 0),
            IntPoint::new(10, 0),
            IntPoint::new(10, 10),
            IntPoint::new(0, 10),
        ];
        let cw = vec![
            IntPoint::new(0, 0),
            IntPoint::new(0, 10),
            IntPoint::new(10, 10),
            IntPoint::new(10, 0),
        ];

        assert_eq!(area(&ccw), 100.0);
        assert!(orientation(&ccw));
        assert_eq!(area(&cw), -100.0);
        assert!(!orientation(&cw));
    }

    #[test]
    fn point_in_polygon_reports_inside_outside_and_boundary() {
        let square = vec![
            IntPoint::new(0, 0),
            IntPoint::new(10, 0),
            IntPoint::new(10, 10),
            IntPoint::new(0, 10),
        ];

        assert_eq!(point_in_polygon(IntPoint::new(5, 5), &square), 1);
        assert_eq!(point_in_polygon(IntPoint::new(15, 5), &square), 0);
        assert_eq!(point_in_polygon(IntPoint::new(10, 5), &square), -1);
        assert_eq!(point_in_polygon(IntPoint::new(0, 0), &square), -1);
    }

    #[test]
    fn out_pt_point_queries_match_path_polygon_queries() {
        let mut square = unsafe {
            out_pt_ring(&[
                IntPoint::new(0, 0),
                IntPoint::new(10, 0),
                IntPoint::new(10, 10),
                IntPoint::new(0, 10),
            ])
        };

        unsafe {
            assert!(point_is_vertex(IntPoint::new(10, 10), square));
            assert!(!point_is_vertex(IntPoint::new(5, 5), square));
            assert_eq!(point_in_polygon_out_pt(IntPoint::new(5, 5), square), 1);
            assert_eq!(point_in_polygon_out_pt(IntPoint::new(15, 5), square), 0);
            assert_eq!(point_in_polygon_out_pt(IntPoint::new(10, 5), square), -1);

            dispose_out_pts(&mut square);
        }
    }

    #[test]
    fn poly2_contains_poly1_uses_first_non_boundary_point() {
        let mut inner = unsafe {
            out_pt_ring(&[
                IntPoint::new(2, 2),
                IntPoint::new(4, 2),
                IntPoint::new(4, 4),
                IntPoint::new(2, 4),
            ])
        };
        let mut outer = unsafe {
            out_pt_ring(&[
                IntPoint::new(0, 0),
                IntPoint::new(10, 0),
                IntPoint::new(10, 10),
                IntPoint::new(0, 10),
            ])
        };
        let mut outside = unsafe {
            out_pt_ring(&[
                IntPoint::new(20, 20),
                IntPoint::new(30, 20),
                IntPoint::new(30, 30),
                IntPoint::new(20, 30),
            ])
        };

        unsafe {
            assert!(poly2_contains_poly1(inner, outer));
            assert!(!poly2_contains_poly1(outside, outer));

            dispose_out_pts(&mut inner);
            dispose_out_pts(&mut outer);
            dispose_out_pts(&mut outside);
        }
    }

    #[test]
    fn slopes_equal_uses_i128_for_full_range() {
        let a = IntPoint::new(0, 0);
        let b = IntPoint::new(3_000_000_000, 3_000_000_000);
        let c = IntPoint::new(6_000_000_000, 6_000_000_000);

        assert!(slopes_equal_3_points(a, b, c, true));
    }

    #[test]
    fn get_dx_and_top_x_match_horizontal_and_sloped_edges() {
        assert_eq!(
            get_dx(IntPoint::new(0, 5), IntPoint::new(10, 5)),
            HORIZONTAL
        );

        let mut edge = TEdge {
            bot: IntPoint::new(0, 0),
            top: IntPoint::new(10, 10),
            ..TEdge::default()
        };
        set_dx(&mut edge);
        assert_eq!(edge.dx, 1.0);
        assert_eq!(top_x(&edge, 7), 7);
        assert_eq!(top_x(&edge, 10), 10);
    }

    #[test]
    fn out_pt_ring_area_reverse_and_bottom_pt_work() {
        let mut ring = unsafe {
            out_pt_ring(&[
                IntPoint::new(0, 0),
                IntPoint::new(10, 0),
                IntPoint::new(10, 10),
                IntPoint::new(0, 10),
            ])
        };

        unsafe {
            assert_eq!(area_out_pt(ring), -100.0);
            assert_eq!((*get_bottom_pt(ring)).pt, IntPoint::new(0, 10));

            let old_prev = (*ring).prev;
            reverse_poly_pt_links(ring);
            assert_eq!((*ring).next, old_prev);

            dispose_out_pts(&mut ring);
        }
        assert!(ring.is_null());
    }

    #[test]
    fn edge_initialization_and_removal_match_linked_list_behavior() {
        let mut edges = vec![TEdge::default(), TEdge::default(), TEdge::default()];
        let e0 = &mut edges[0] as *mut TEdge;
        let e1 = &mut edges[1] as *mut TEdge;
        let e2 = &mut edges[2] as *mut TEdge;

        unsafe {
            init_edge(e0, e1, e2, IntPoint::new(0, 10));
            init_edge(e1, e2, e0, IntPoint::new(10, 0));
            init_edge(e2, e0, e1, IntPoint::new(20, 10));

            init_edge2(&mut *e0, PolyType::Subject);
            assert_eq!((*e0).bot, IntPoint::new(0, 10));
            assert_eq!((*e0).top, IntPoint::new(10, 0));
            assert_eq!((*e0).out_idx, UNASSIGNED);

            let next = remove_edge(e1);
            assert_eq!(next, e2);
            assert_eq!((*e0).next, e2);
            assert_eq!((*e2).prev, e0);
            assert!((*e1).prev.is_null());
        }
    }

    #[test]
    fn overlap_helpers_match_open_interval_behavior() {
        assert!(horz_segments_overlap(0, 10, 5, 15));
        assert!(!horz_segments_overlap(0, 10, 10, 15));
        assert_eq!(get_overlap(0, 10, 5, 15), Some((5, 10)));
        assert_eq!(get_overlap(0, 10, 10, 15), None);

        let mut left = IntPoint::default();
        let mut right = IntPoint::default();
        assert!(get_overlap_segment(
            IntPoint::new(0, 0),
            IntPoint::new(10, 0),
            IntPoint::new(4, 0),
            IntPoint::new(12, 0),
            &mut left,
            &mut right,
        ));
        assert_eq!(left, IntPoint::new(4, 0));
        assert_eq!(right, IntPoint::new(10, 0));
    }

    #[test]
    fn poly_tree_tracks_children_and_hole_depth() {
        let mut tree = PolyTree::new();
        let child = Box::into_raw(Box::new(PolyNode::new()));
        let grandchild = Box::into_raw(Box::new(PolyNode::new()));
        tree.all_nodes.push(child);
        tree.all_nodes.push(grandchild);

        unsafe {
            tree.node.add_child(child);
            (*child).add_child(grandchild);

            assert_eq!(tree.get_first(), child);
            assert_eq!(tree.total(), 2);
            assert_eq!((*child).child_count(), 1);
            assert!(!(*child).is_hole());
            assert!((*grandchild).is_hole());
            assert_eq!((*child).get_next(), grandchild);
            assert!((*grandchild).get_next().is_null());
        }
    }

    #[test]
    fn range_test_switches_to_full_range_then_rejects_huge_coordinates() {
        let mut full_range = false;

        range_test(IntPoint::new(LO_RANGE + 1, 0), &mut full_range).unwrap();
        assert!(full_range);

        let err = range_test(IntPoint::new(HI_RANGE + 1, 0), &mut full_range).unwrap_err();
        assert_eq!(err.to_string(), "Coordinate outside allowed range");
    }

    #[test]
    fn unit_normal_matches_clipper_orientation() {
        let normal = get_unit_normal(IntPoint::new(0, 0), IntPoint::new(10, 0));
        assert_eq!(normal, DoublePoint::new(0.0, -1.0));

        let normal = get_unit_normal(IntPoint::new(0, 0), IntPoint::new(0, 10));
        assert_eq!(normal, DoublePoint::new(1.0, -0.0));

        assert_eq!(
            get_unit_normal(IntPoint::new(3, 3), IntPoint::new(3, 3)),
            DoublePoint::new(0.0, 0.0)
        );
    }

    #[test]
    fn reverse_and_translate_paths_work_in_place_style() {
        let mut path = vec![
            IntPoint::new(0, 0),
            IntPoint::new(1, 0),
            IntPoint::new(1, 1),
        ];
        reverse_path(&mut path);
        assert_eq!(
            path,
            vec![
                IntPoint::new(1, 1),
                IntPoint::new(1, 0),
                IntPoint::new(0, 0),
            ]
        );

        let mut translated = Vec::new();
        translate_path(&path, &mut translated, IntPoint::new(10, -2));
        assert_eq!(
            translated,
            vec![
                IntPoint::new(11, -1),
                IntPoint::new(11, -2),
                IntPoint::new(10, -2),
            ]
        );
    }

    #[test]
    fn clean_polygon_removes_duplicate_and_near_collinear_points() {
        let path = vec![
            IntPoint::new(0, 0),
            IntPoint::new(5, 0),
            IntPoint::new(10, 0),
            IntPoint::new(10, 10),
            IntPoint::new(0, 10),
            IntPoint::new(0, 10),
        ];

        let cleaned = clean_polygon(&path, 1.415);

        assert_eq!(
            cleaned,
            vec![
                IntPoint::new(0, 0),
                IntPoint::new(10, 0),
                IntPoint::new(10, 10),
                IntPoint::new(0, 10),
            ]
        );
    }

    #[test]
    fn distance_helpers_identify_close_and_near_collinear_points() {
        assert_eq!(
            distance_sqrd(IntPoint::new(0, 0), IntPoint::new(3, 4)),
            25.0
        );
        assert_eq!(
            distance_from_line_sqrd(
                IntPoint::new(5, 1),
                IntPoint::new(0, 0),
                IntPoint::new(10, 0)
            ),
            1.0
        );
        assert!(points_are_close(
            IntPoint::new(0, 0),
            IntPoint::new(1, 1),
            2.0
        ));
        assert!(slopes_near_collinear(
            IntPoint::new(0, 0),
            IntPoint::new(5, 1),
            IntPoint::new(10, 0),
            2.0
        ));
    }

    #[test]
    fn minkowski_builds_oriented_quads() {
        let pattern = vec![
            IntPoint::new(0, 0),
            IntPoint::new(1, 0),
            IntPoint::new(1, 1),
        ];
        let path = vec![
            IntPoint::new(10, 10),
            IntPoint::new(20, 10),
            IntPoint::new(20, 20),
        ];
        let mut solution = Vec::new();

        minkowski(&pattern, &path, &mut solution, true, false);

        assert_eq!(solution.len(), 6);
        assert!(solution.iter().all(orientation));
    }

    #[test]
    fn simplify_polygon_uses_strict_simple_union() {
        let bow_tie = vec![
            IntPoint::new(0, 0),
            IntPoint::new(10, 10),
            IntPoint::new(0, 10),
            IntPoint::new(10, 0),
        ];

        let simplified = simplify_polygon(&bow_tie, PolyFillType::EvenOdd).unwrap();

        assert!(!simplified.is_empty());
    }

    #[test]
    fn minkowski_wrappers_union_generated_quads() {
        let pattern = vec![
            IntPoint::new(0, 0),
            IntPoint::new(2, 0),
            IntPoint::new(2, 2),
            IntPoint::new(0, 2),
        ];
        let path = vec![
            IntPoint::new(0, 0),
            IntPoint::new(10, 0),
            IntPoint::new(10, 10),
            IntPoint::new(0, 10),
        ];
        let mut sum = Vec::new();
        let mut diff = Vec::new();

        minkowski_sum_into(&pattern, &path, &mut sum, true).unwrap();
        minkowski_diff_into(&pattern, &path, &mut diff).unwrap();

        assert!(!sum.is_empty());
        assert!(!diff.is_empty());
    }

    #[test]
    fn polytree_path_extractors_filter_open_and_closed_nodes() {
        let mut polytree = PolyTree::new();
        let closed = Box::into_raw(Box::new(PolyNode {
            contour: vec![
                IntPoint::new(0, 0),
                IntPoint::new(10, 0),
                IntPoint::new(10, 10),
            ],
            ..PolyNode::new()
        }));
        let open = Box::into_raw(Box::new(PolyNode {
            contour: vec![IntPoint::new(20, 20), IntPoint::new(30, 30)],
            is_open: true,
            ..PolyNode::new()
        }));

        unsafe {
            polytree.all_nodes.push(closed);
            polytree.all_nodes.push(open);
            polytree.node.add_child(closed);
            polytree.node.add_child(open);
        }

        let mut all = Vec::new();
        let mut closed_paths = Vec::new();
        let mut open_paths = Vec::new();

        poly_tree_to_paths(&polytree, &mut all);
        closed_paths_from_poly_tree(&polytree, &mut closed_paths);
        open_paths_from_poly_tree(&polytree, &mut open_paths);

        assert_eq!(all.len(), 2);
        assert_eq!(closed_paths.len(), 1);
        assert_eq!(open_paths.len(), 1);
        assert_eq!(
            open_paths[0],
            vec![IntPoint::new(20, 20), IntPoint::new(30, 30)]
        );
    }

    unsafe fn out_pt_ring(points: &[IntPoint]) -> *mut OutPt {
        assert!(points.len() >= 3);
        let mut raw = Vec::new();
        for (idx, point) in points.iter().enumerate() {
            raw.push(Box::into_raw(Box::new(OutPt {
                idx: idx as i32,
                pt: *point,
                next: std::ptr::null_mut(),
                prev: std::ptr::null_mut(),
            })));
        }

        for i in 0..raw.len() {
            let prev = if i == 0 { raw.len() - 1 } else { i - 1 };
            let next = if i == raw.len() - 1 { 0 } else { i + 1 };
            // SAFETY: raw entries came from Box::into_raw and remain alive for the ring.
            unsafe {
                (*raw[i]).prev = raw[prev];
                (*raw[i]).next = raw[next];
            }
        }

        raw[0]
    }
}
