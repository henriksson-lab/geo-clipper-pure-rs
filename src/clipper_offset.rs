use crate::clipper::Clipper;
use crate::error::Result;
use crate::helpers::{get_unit_normal, near_zero, orientation, reverse_path, round};
use crate::types::{
    AddPathResult, ClipType, DEF_ARC_TOLERANCE, DoublePoint, EndType, IntPoint, JoinType,
    Orientation, PI, Path, Paths, PolyFillType, PolyNode, PolyTree, PolyType, TWO_PI,
};

#[derive(Debug)]
pub struct ClipperOffset {
    pub(crate) miter_limit: f64,
    pub(crate) arc_tolerance: f64,
    pub(crate) dest_polys: Paths,
    pub(crate) src_poly: Path,
    pub(crate) dest_poly: Path,
    pub(crate) normals: Vec<DoublePoint>,
    pub(crate) delta: f64,
    pub(crate) sin_a: f64,
    pub(crate) sin: f64,
    pub(crate) cos: f64,
    pub(crate) miter_lim: f64,
    pub(crate) steps_per_rad: f64,
    pub(crate) lowest: IntPoint,
    pub(crate) poly_nodes: PolyNode,
}

impl Default for ClipperOffset {
    fn default() -> Self {
        Self::new(2.0, 0.25)
    }
}

impl ClipperOffset {
    // C++: ClipperOffset::ClipperOffset
    pub fn new(miter_limit: f64, arc_tolerance: f64) -> Self {
        Self {
            miter_limit,
            arc_tolerance,
            dest_polys: Paths::new(),
            src_poly: Path::new(),
            dest_poly: Path::new(),
            normals: Vec::new(),
            delta: 0.0,
            sin_a: 0.0,
            sin: 0.0,
            cos: 0.0,
            miter_lim: 0.0,
            steps_per_rad: 0.0,
            lowest: IntPoint::new(-1, 0),
            poly_nodes: PolyNode::new(),
        }
    }

    // C++: ClipperOffset::Clear
    pub fn clear(&mut self) {
        for child in self.poly_nodes.childs.drain(..) {
            if !child.is_null() {
                unsafe {
                    drop(Box::from_raw(child));
                }
            }
        }
        self.lowest.x = -1;
    }

    // C++: ClipperOffset::AddPath
    pub fn add_path(
        &mut self,
        path: &[IntPoint],
        join_type: JoinType,
        end_type: EndType,
    ) -> AddPathResult {
        let mut high_i = path.len();
        if high_i == 0 {
            return AddPathResult::Skipped;
        }
        high_i -= 1;

        let mut new_node = Box::new(PolyNode::new());
        new_node.jointype = join_type;
        new_node.endtype = end_type;

        if end_type == EndType::ClosedLine || end_type == EndType::ClosedPolygon {
            while high_i > 0 && path[0] == path[high_i] {
                high_i -= 1;
            }
        }

        new_node.contour.reserve(high_i + 1);
        new_node.contour.push(path[0]);
        let mut j = 0usize;
        let mut k = 0usize;
        for i in 1..=high_i {
            if new_node.contour[j] != path[i] {
                j += 1;
                new_node.contour.push(path[i]);
                if path[i].y > new_node.contour[k].y
                    || (path[i].y == new_node.contour[k].y && path[i].x < new_node.contour[k].x)
                {
                    k = j;
                }
            }
        }

        if end_type == EndType::ClosedPolygon && j < 2 {
            return AddPathResult::Skipped;
        }

        let new_node_ptr = Box::into_raw(new_node);
        unsafe {
            self.poly_nodes.add_child(new_node_ptr);
        }

        if end_type != EndType::ClosedPolygon {
            return AddPathResult::Added;
        }
        if self.lowest.x < 0 {
            self.lowest = IntPoint::new((self.poly_nodes.child_count() - 1) as i64, k as i64);
        } else {
            let ip = unsafe {
                (&(*self.poly_nodes.childs[self.lowest.x as usize]).contour)[self.lowest.y as usize]
            };
            let new_low = unsafe { (&(*new_node_ptr).contour)[k] };
            if new_low.y > ip.y || (new_low.y == ip.y && new_low.x < ip.x) {
                self.lowest = IntPoint::new((self.poly_nodes.child_count() - 1) as i64, k as i64);
            }
        }
        AddPathResult::Added
    }

    // C++: ClipperOffset::AddPaths
    pub fn add_paths(
        &mut self,
        paths: &[Path],
        join_type: JoinType,
        end_type: EndType,
    ) -> AddPathResult {
        let mut result = AddPathResult::Skipped;
        for path in paths {
            if self.add_path(path, join_type, end_type).was_added() {
                result = AddPathResult::Added;
            }
        }
        result
    }

    // C++: ClipperOffset::FixOrientations
    unsafe fn fix_orientations(&mut self) {
        unsafe {
            if self.lowest.x >= 0
                && orientation(&(*self.poly_nodes.childs[self.lowest.x as usize]).contour)
                    == Orientation::Clockwise
            {
                for i in 0..self.poly_nodes.child_count() {
                    let node = &mut *self.poly_nodes.childs[i];
                    if node.endtype == EndType::ClosedPolygon
                        || (node.endtype == EndType::ClosedLine
                            && orientation(&node.contour) == Orientation::CounterClockwise)
                    {
                        reverse_path(&mut node.contour);
                    }
                }
            } else {
                for i in 0..self.poly_nodes.child_count() {
                    let node = &mut *self.poly_nodes.childs[i];
                    if node.endtype == EndType::ClosedLine
                        && orientation(&node.contour) == Orientation::Clockwise
                    {
                        reverse_path(&mut node.contour);
                    }
                }
            }
        }
    }

    // C++: ClipperOffset::Execute(Paths&, double)
    pub fn execute(&mut self, delta: f64) -> Result<Paths> {
        let mut solution = Paths::new();
        self.execute_into(&mut solution, delta)?;
        Ok(solution)
    }

    pub fn execute_into(&mut self, solution: &mut Paths, delta: f64) -> Result<()> {
        solution.clear();
        unsafe {
            self.fix_orientations();
            self.do_offset(delta);
        }

        let mut clpr = Clipper::new();
        clpr.add_paths(&self.dest_polys, PolyType::Subject, true)?;
        if delta > 0.0 {
            *solution = clpr.execute_with_fill_types(
                ClipType::Union,
                PolyFillType::Positive,
                PolyFillType::Positive,
            )?;
        } else {
            let r = clpr.bounds();
            let outer = vec![
                IntPoint::new(r.left - 10, r.bottom + 10),
                IntPoint::new(r.right + 10, r.bottom + 10),
                IntPoint::new(r.right + 10, r.top - 10),
                IntPoint::new(r.left - 10, r.top - 10),
            ];

            clpr.add_path(&outer, PolyType::Subject, true)?;
            clpr.set_reverse_solution(true);
            *solution = clpr.execute_with_fill_types(
                ClipType::Union,
                PolyFillType::Negative,
                PolyFillType::Negative,
            )?;
            if !solution.is_empty() {
                solution.remove(0);
            }
        }
        Ok(())
    }

    // C++: ClipperOffset::Execute(PolyTree&, double)
    pub fn execute_polytree(&mut self, delta: f64) -> Result<PolyTree> {
        let mut solution = PolyTree::new();
        self.execute_polytree_into(&mut solution, delta)?;
        Ok(solution)
    }

    pub fn execute_polytree_into(&mut self, solution: &mut PolyTree, delta: f64) -> Result<()> {
        unsafe {
            solution.clear();
            self.fix_orientations();
            self.do_offset(delta);
        }

        let mut clpr = Clipper::new();
        clpr.add_paths(&self.dest_polys, PolyType::Subject, true)?;
        if delta > 0.0 {
            *solution = clpr.execute_polytree_with_fill_types(
                ClipType::Union,
                PolyFillType::Positive,
                PolyFillType::Positive,
            )?;
        } else {
            let r = clpr.bounds();
            let outer = vec![
                IntPoint::new(r.left - 10, r.bottom + 10),
                IntPoint::new(r.right + 10, r.bottom + 10),
                IntPoint::new(r.right + 10, r.top - 10),
                IntPoint::new(r.left - 10, r.top - 10),
            ];

            clpr.add_path(&outer, PolyType::Subject, true)?;
            clpr.set_reverse_solution(true);
            *solution = clpr.execute_polytree_with_fill_types(
                ClipType::Union,
                PolyFillType::Negative,
                PolyFillType::Negative,
            )?;

            unsafe {
                if solution.node.child_count() == 1 && (*solution.node.childs[0]).child_count() > 0
                {
                    let outer_node = solution.node.childs[0];
                    solution.node.childs.clear();
                    solution.node.childs.reserve((*outer_node).child_count());
                    let first_child = (&(*outer_node).childs)[0];
                    solution.node.childs.push(first_child);
                    (*first_child).parent = &mut solution.node;
                    (*first_child).index = 0;
                    for i in 1..(*outer_node).child_count() {
                        solution.node.add_child((&(*outer_node).childs)[i]);
                    }
                } else {
                    solution.clear();
                }
            }
        }
        Ok(())
    }

    // C++: ClipperOffset::DoOffset
    unsafe fn do_offset(&mut self, delta: f64) {
        self.dest_polys.clear();
        self.delta = delta;

        if near_zero(delta) {
            self.dest_polys.reserve(self.poly_nodes.child_count());
            for i in 0..self.poly_nodes.child_count() {
                unsafe {
                    let node = &*self.poly_nodes.childs[i];
                    if node.endtype == EndType::ClosedPolygon {
                        self.dest_polys.push(node.contour.clone());
                    }
                }
            }
            return;
        }

        if self.miter_limit > 2.0 {
            self.miter_lim = 2.0 / (self.miter_limit * self.miter_limit);
        } else {
            self.miter_lim = 0.5;
        }

        let y = if self.arc_tolerance <= 0.0 {
            DEF_ARC_TOLERANCE
        } else if self.arc_tolerance > delta.abs() * DEF_ARC_TOLERANCE {
            delta.abs() * DEF_ARC_TOLERANCE
        } else {
            self.arc_tolerance
        };
        let mut steps = PI / (1.0 - y / delta.abs()).acos();
        if steps > delta.abs() * PI {
            steps = delta.abs() * PI;
        }
        self.sin = (TWO_PI / steps).sin();
        self.cos = (TWO_PI / steps).cos();
        self.steps_per_rad = steps / TWO_PI;
        if delta < 0.0 {
            self.sin = -self.sin;
        }

        self.dest_polys.reserve(self.poly_nodes.child_count() * 2);
        for i in 0..self.poly_nodes.child_count() {
            unsafe {
                let node = &*self.poly_nodes.childs[i];
                self.src_poly.clear();
                self.src_poly.extend_from_slice(&node.contour);

                let len = self.src_poly.len();
                if len == 0 || (delta <= 0.0 && (len < 3 || node.endtype != EndType::ClosedPolygon))
                {
                    continue;
                }

                self.dest_poly.clear();
                if len == 1 {
                    if node.jointype == JoinType::Round {
                        let mut x = 1.0;
                        let mut y = 0.0;
                        for _ in 1..=(steps as i64) {
                            self.dest_poly.push(IntPoint::new(
                                round(self.src_poly[0].x as f64 + x * delta),
                                round(self.src_poly[0].y as f64 + y * delta),
                            ));
                            let x2 = x;
                            x = x * self.cos - self.sin * y;
                            y = x2 * self.sin + y * self.cos;
                        }
                    } else {
                        let mut x = -1.0;
                        let mut y = -1.0;
                        for _ in 0..4 {
                            self.dest_poly.push(IntPoint::new(
                                round(self.src_poly[0].x as f64 + x * delta),
                                round(self.src_poly[0].y as f64 + y * delta),
                            ));
                            if x < 0.0 {
                                x = 1.0;
                            } else if y < 0.0 {
                                y = 1.0;
                            } else {
                                x = -1.0;
                            }
                        }
                    }
                    self.dest_polys.push(self.dest_poly.clone());
                    continue;
                }

                self.normals.clear();
                self.normals.reserve(len);
                for j in 0..(len - 1) {
                    self.normals
                        .push(get_unit_normal(self.src_poly[j], self.src_poly[j + 1]));
                }
                if node.endtype == EndType::ClosedLine || node.endtype == EndType::ClosedPolygon {
                    self.normals
                        .push(get_unit_normal(self.src_poly[len - 1], self.src_poly[0]));
                } else {
                    self.normals.push(self.normals[len - 2]);
                }

                if node.endtype == EndType::ClosedPolygon {
                    let mut k = len - 1;
                    for j in 0..len {
                        self.offset_point(j, &mut k, node.jointype);
                    }
                    self.dest_polys.push(self.dest_poly.clone());
                } else if node.endtype == EndType::ClosedLine {
                    let mut k = len - 1;
                    for j in 0..len {
                        self.offset_point(j, &mut k, node.jointype);
                    }
                    self.dest_polys.push(self.dest_poly.clone());
                    self.dest_poly.clear();

                    let n = self.normals[len - 1];
                    for j in (1..len).rev() {
                        self.normals[j] =
                            DoublePoint::new(-self.normals[j - 1].x, -self.normals[j - 1].y);
                    }
                    self.normals[0] = DoublePoint::new(-n.x, -n.y);
                    k = 0;
                    for j in (0..len).rev() {
                        self.offset_point(j, &mut k, node.jointype);
                    }
                    self.dest_polys.push(self.dest_poly.clone());
                } else {
                    let mut k = 0;
                    for j in 1..(len - 1) {
                        self.offset_point(j, &mut k, node.jointype);
                    }

                    if node.endtype == EndType::OpenButt {
                        let j = len - 1;
                        self.dest_poly.push(IntPoint::new(
                            round(self.src_poly[j].x as f64 + self.normals[j].x * delta),
                            round(self.src_poly[j].y as f64 + self.normals[j].y * delta),
                        ));
                        self.dest_poly.push(IntPoint::new(
                            round(self.src_poly[j].x as f64 - self.normals[j].x * delta),
                            round(self.src_poly[j].y as f64 - self.normals[j].y * delta),
                        ));
                    } else {
                        let j = len - 1;
                        k = len - 2;
                        self.sin_a = 0.0;
                        self.normals[j] = DoublePoint::new(-self.normals[j].x, -self.normals[j].y);
                        if node.endtype == EndType::OpenSquare {
                            self.do_square(j, k);
                        } else {
                            self.do_round(j, k);
                        }
                    }

                    for j in (1..len).rev() {
                        self.normals[j] =
                            DoublePoint::new(-self.normals[j - 1].x, -self.normals[j - 1].y);
                    }
                    self.normals[0] = DoublePoint::new(-self.normals[1].x, -self.normals[1].y);

                    k = len - 1;
                    for j in (1..k).rev() {
                        self.offset_point(j, &mut k, node.jointype);
                    }

                    if node.endtype == EndType::OpenButt {
                        self.dest_poly.push(IntPoint::new(
                            round(self.src_poly[0].x as f64 - self.normals[0].x * delta),
                            round(self.src_poly[0].y as f64 - self.normals[0].y * delta),
                        ));
                        self.dest_poly.push(IntPoint::new(
                            round(self.src_poly[0].x as f64 + self.normals[0].x * delta),
                            round(self.src_poly[0].y as f64 + self.normals[0].y * delta),
                        ));
                    } else {
                        self.sin_a = 0.0;
                        if node.endtype == EndType::OpenSquare {
                            self.do_square(0, 1);
                        } else {
                            self.do_round(0, 1);
                        }
                    }
                    self.dest_polys.push(self.dest_poly.clone());
                }
            }
        }
    }

    // C++: ClipperOffset::OffsetPoint
    fn offset_point(&mut self, j: usize, k: &mut usize, jointype: JoinType) {
        self.sin_a =
            self.normals[*k].x * self.normals[j].y - self.normals[j].x * self.normals[*k].y;
        if (self.sin_a * self.delta).abs() < 1.0 {
            let cos_a =
                self.normals[*k].x * self.normals[j].x + self.normals[j].y * self.normals[*k].y;
            if cos_a > 0.0 {
                self.dest_poly.push(IntPoint::new(
                    round(self.src_poly[j].x as f64 + self.normals[*k].x * self.delta),
                    round(self.src_poly[j].y as f64 + self.normals[*k].y * self.delta),
                ));
                return;
            }
        } else if self.sin_a > 1.0 {
            self.sin_a = 1.0;
        } else if self.sin_a < -1.0 {
            self.sin_a = -1.0;
        }

        if self.sin_a * self.delta < 0.0 {
            self.dest_poly.push(IntPoint::new(
                round(self.src_poly[j].x as f64 + self.normals[*k].x * self.delta),
                round(self.src_poly[j].y as f64 + self.normals[*k].y * self.delta),
            ));
            self.dest_poly.push(self.src_poly[j]);
            self.dest_poly.push(IntPoint::new(
                round(self.src_poly[j].x as f64 + self.normals[j].x * self.delta),
                round(self.src_poly[j].y as f64 + self.normals[j].y * self.delta),
            ));
        } else {
            match jointype {
                JoinType::Miter => {
                    let r = 1.0
                        + (self.normals[j].x * self.normals[*k].x
                            + self.normals[j].y * self.normals[*k].y);
                    if r >= self.miter_lim {
                        self.do_miter(j, *k, r);
                    } else {
                        self.do_square(j, *k);
                    }
                }
                JoinType::Square => self.do_square(j, *k),
                JoinType::Round => self.do_round(j, *k),
            }
        }
        *k = j;
    }

    // C++: ClipperOffset::DoSquare
    fn do_square(&mut self, j: usize, k: usize) {
        let dx = (self.sin_a)
            .atan2(self.normals[k].x * self.normals[j].x + self.normals[k].y * self.normals[j].y)
            / 4.0;
        let dx = dx.tan();
        self.dest_poly.push(IntPoint::new(
            round(
                self.src_poly[j].x as f64
                    + self.delta * (self.normals[k].x - self.normals[k].y * dx),
            ),
            round(
                self.src_poly[j].y as f64
                    + self.delta * (self.normals[k].y + self.normals[k].x * dx),
            ),
        ));
        self.dest_poly.push(IntPoint::new(
            round(
                self.src_poly[j].x as f64
                    + self.delta * (self.normals[j].x + self.normals[j].y * dx),
            ),
            round(
                self.src_poly[j].y as f64
                    + self.delta * (self.normals[j].y - self.normals[j].x * dx),
            ),
        ));
    }

    // C++: ClipperOffset::DoMiter
    fn do_miter(&mut self, j: usize, k: usize, r: f64) {
        let q = self.delta / r;
        self.dest_poly.push(IntPoint::new(
            round(self.src_poly[j].x as f64 + (self.normals[k].x + self.normals[j].x) * q),
            round(self.src_poly[j].y as f64 + (self.normals[k].y + self.normals[j].y) * q),
        ));
    }

    // C++: ClipperOffset::DoRound
    fn do_round(&mut self, j: usize, k: usize) {
        let a = self
            .sin_a
            .atan2(self.normals[k].x * self.normals[j].x + self.normals[k].y * self.normals[j].y);
        let steps = round(self.steps_per_rad * a.abs()).max(1);

        let mut x = self.normals[k].x;
        let mut y = self.normals[k].y;
        for _ in 0..steps {
            self.dest_poly.push(IntPoint::new(
                round(self.src_poly[j].x as f64 + x * self.delta),
                round(self.src_poly[j].y as f64 + y * self.delta),
            ));
            let x2 = x;
            x = x * self.cos - self.sin * y;
            y = x2 * self.sin + y * self.cos;
        }
        self.dest_poly.push(IntPoint::new(
            round(self.src_poly[j].x as f64 + self.normals[j].x * self.delta),
            round(self.src_poly[j].y as f64 + self.normals[j].y * self.delta),
        ));
    }
}

impl Drop for ClipperOffset {
    fn drop(&mut self) {
        self.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_path_strips_duplicate_endpoints_and_tracks_lowest_point() {
        let mut offset = ClipperOffset::new(2.0, 0.25);
        let path = vec![
            IntPoint::new(0, 0),
            IntPoint::new(10, 0),
            IntPoint::new(10, 10),
            IntPoint::new(0, 10),
            IntPoint::new(0, 0),
        ];

        unsafe {
            offset.add_path(&path, JoinType::Miter, EndType::ClosedPolygon);

            assert_eq!(offset.poly_nodes.child_count(), 1);
            let node = &*offset.poly_nodes.childs[0];
            assert_eq!(node.contour.len(), 4);
            assert_eq!(node.jointype, JoinType::Miter);
            assert_eq!(node.endtype, EndType::ClosedPolygon);
            assert_eq!(offset.lowest, IntPoint::new(0, 3));
        }
    }

    #[test]
    fn clear_drops_offset_nodes_and_resets_lowest() {
        let mut offset = ClipperOffset::new(2.0, 0.25);
        offset.add_path(
            &vec![
                IntPoint::new(0, 0),
                IntPoint::new(10, 0),
                IntPoint::new(10, 10),
            ],
            JoinType::Square,
            EndType::ClosedPolygon,
        );
        offset.clear();

        assert_eq!(offset.poly_nodes.child_count(), 0);
        assert_eq!(offset.lowest.x, -1);
    }

    #[test]
    fn fix_orientations_reverses_closed_line_when_needed() {
        let mut offset = ClipperOffset::new(2.0, 0.25);
        unsafe {
            offset.add_path(
                &vec![
                    IntPoint::new(0, 0),
                    IntPoint::new(0, 10),
                    IntPoint::new(10, 10),
                    IntPoint::new(10, 0),
                ],
                JoinType::Square,
                EndType::ClosedLine,
            );
            assert_eq!(
                orientation(&(*offset.poly_nodes.childs[0]).contour),
                Orientation::Clockwise
            );

            offset.fix_orientations();

            assert_eq!(
                orientation(&(*offset.poly_nodes.childs[0]).contour),
                Orientation::CounterClockwise
            );
        }
    }

    #[test]
    fn zero_do_offset_copies_closed_polygons_only() {
        let mut offset = ClipperOffset::new(2.0, 0.25);
        unsafe {
            offset.add_path(
                &vec![
                    IntPoint::new(0, 0),
                    IntPoint::new(10, 0),
                    IntPoint::new(10, 10),
                ],
                JoinType::Square,
                EndType::ClosedPolygon,
            );
            offset.add_path(
                &vec![
                    IntPoint::new(20, 0),
                    IntPoint::new(30, 0),
                    IntPoint::new(30, 10),
                ],
                JoinType::Square,
                EndType::OpenButt,
            );

            offset.do_offset(0.0);

            assert_eq!(offset.dest_polys.len(), 1);
            assert_eq!(offset.dest_polys[0].len(), 3);
        }
    }

    #[test]
    fn do_offset_expands_square_with_miter_joins() {
        let mut offset = ClipperOffset::new(2.0, 0.25);
        unsafe {
            offset.add_path(
                &vec![
                    IntPoint::new(0, 0),
                    IntPoint::new(10, 0),
                    IntPoint::new(10, 10),
                    IntPoint::new(0, 10),
                ],
                JoinType::Miter,
                EndType::ClosedPolygon,
            );
            offset.fix_orientations();
            offset.do_offset(1.0);

            assert_eq!(offset.dest_polys.len(), 1);
            assert_eq!(
                offset.dest_polys[0],
                vec![
                    IntPoint::new(-1, -1),
                    IntPoint::new(11, -1),
                    IntPoint::new(11, 11),
                    IntPoint::new(-1, 11),
                ]
            );
        }
    }

    #[test]
    fn execute_positive_offset_cleans_expanded_square() {
        let mut offset = ClipperOffset::new(2.0, 0.25);
        offset.add_path(
            &vec![
                IntPoint::new(0, 0),
                IntPoint::new(10, 0),
                IntPoint::new(10, 10),
                IntPoint::new(0, 10),
            ],
            JoinType::Miter,
            EndType::ClosedPolygon,
        );
        let solution = offset.execute(1.0).unwrap();

        assert_eq!(solution.len(), 1);
        assert_eq!(crate::helpers::area(&solution[0]).abs(), 144.0);
    }

    #[test]
    fn execute_polytree_positive_offset_builds_node() {
        let mut offset = ClipperOffset::new(2.0, 0.25);
        offset.add_path(
            &vec![
                IntPoint::new(0, 0),
                IntPoint::new(10, 0),
                IntPoint::new(10, 10),
                IntPoint::new(0, 10),
            ],
            JoinType::Miter,
            EndType::ClosedPolygon,
        );
        let solution = offset.execute_polytree(1.0).unwrap();

        assert_eq!(solution.total(), 1);
        unsafe {
            let first = solution.get_first();
            assert!(!(*first).is_open());
            assert_eq!(crate::helpers::area(&(*first).contour).abs(), 144.0);
        }
    }

    #[test]
    fn execute_negative_offset_shrinks_square() {
        let mut offset = ClipperOffset::new(2.0, 0.25);
        offset.add_path(
            &vec![
                IntPoint::new(0, 0),
                IntPoint::new(10, 0),
                IntPoint::new(10, 10),
                IntPoint::new(0, 10),
            ],
            JoinType::Miter,
            EndType::ClosedPolygon,
        );
        let solution = offset.execute(-1.0).unwrap();

        assert_eq!(solution.len(), 1);
        assert_eq!(crate::helpers::area(&solution[0]).abs(), 64.0);
    }

    #[test]
    fn execute_polytree_negative_offset_shrinks_square() {
        let mut offset = ClipperOffset::new(2.0, 0.25);
        offset.add_path(
            &vec![
                IntPoint::new(0, 0),
                IntPoint::new(10, 0),
                IntPoint::new(10, 10),
                IntPoint::new(0, 10),
            ],
            JoinType::Miter,
            EndType::ClosedPolygon,
        );
        let solution = offset.execute_polytree(-1.0).unwrap();

        assert_eq!(solution.total(), 1);
        unsafe {
            assert_eq!(
                crate::helpers::area(&(*solution.get_first()).contour).abs(),
                64.0
            );
        }
    }
}
