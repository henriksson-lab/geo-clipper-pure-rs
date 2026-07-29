use std::ptr;

pub type CInt = i64;
pub type Path = Vec<IntPoint>;
pub type Paths = Vec<Path>;

pub const CLIPPER_VERSION: &str = "6.4.2";
pub const LO_RANGE: CInt = 0x3FFF_FFFF;
pub const HI_RANGE: CInt = 0x3FFF_FFFF_FFFF_FFFF;

pub const PI: f64 = 3.141592653589793238;
pub const TWO_PI: f64 = PI * 2.0;
pub const DEF_ARC_TOLERANCE: f64 = 0.25;

pub const HORIZONTAL: f64 = -1.0E40;
pub const TOLERANCE: f64 = 1.0E-20;

pub const UNASSIGNED: i32 = -1;
pub const SKIP: i32 = -2;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum AddPathResult {
    Added,
    Skipped,
}

impl AddPathResult {
    pub fn was_added(self) -> bool {
        self == Self::Added
    }
}

impl From<bool> for AddPathResult {
    fn from(added: bool) -> Self {
        if added { Self::Added } else { Self::Skipped }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Orientation {
    Clockwise,
    CounterClockwise,
}

impl Orientation {
    pub fn is_counter_clockwise(self) -> bool {
        self == Self::CounterClockwise
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PointLocation {
    Outside,
    Inside,
    Boundary,
}

impl PointLocation {
    pub fn from_clipper_code(code: i32) -> Self {
        match code {
            1 => Self::Inside,
            -1 => Self::Boundary,
            _ => Self::Outside,
        }
    }
}

#[derive(Debug, Copy, Clone, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct IntPoint {
    pub x: CInt,
    pub y: CInt,
}

impl IntPoint {
    pub fn new(x: CInt, y: CInt) -> Self {
        Self { x, y }
    }
}

impl From<(CInt, CInt)> for IntPoint {
    fn from((x, y): (CInt, CInt)) -> Self {
        Self { x, y }
    }
}

impl From<IntPoint> for (CInt, CInt) {
    fn from(point: IntPoint) -> Self {
        (point.x, point.y)
    }
}

#[derive(Debug, Copy, Clone, Default, PartialEq)]
pub struct DoublePoint {
    pub x: f64,
    pub y: f64,
}

impl DoublePoint {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

impl From<IntPoint> for DoublePoint {
    fn from(ip: IntPoint) -> Self {
        Self {
            x: ip.x as f64,
            y: ip.y as f64,
        }
    }
}

#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
pub struct IntRect {
    pub left: CInt,
    pub top: CInt,
    pub right: CInt,
    pub bottom: CInt,
}

impl IntRect {
    pub fn new(left: CInt, top: CInt, right: CInt, bottom: CInt) -> Self {
        Self {
            left,
            top,
            right,
            bottom,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ClipType {
    Intersection,
    Union,
    Difference,
    Xor,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PolyType {
    Subject,
    Clip,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PolyFillType {
    EvenOdd,
    NonZero,
    Positive,
    Negative,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum JoinType {
    Square,
    Round,
    Miter,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum EndType {
    ClosedPolygon,
    ClosedLine,
    OpenButt,
    OpenSquare,
    OpenRound,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum EdgeSide {
    Left = 1,
    Right = 2,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Direction {
    RightToLeft,
    LeftToRight,
}

#[derive(Debug)]
pub struct TEdge {
    pub bot: IntPoint,
    pub curr: IntPoint,
    pub top: IntPoint,
    pub dx: f64,
    pub poly_typ: PolyType,
    pub side: EdgeSide,
    pub wind_delta: i32,
    pub wind_cnt: i32,
    pub wind_cnt2: i32,
    pub out_idx: i32,
    pub next: *mut TEdge,
    pub prev: *mut TEdge,
    pub next_in_lml: *mut TEdge,
    pub next_in_ael: *mut TEdge,
    pub prev_in_ael: *mut TEdge,
    pub next_in_sel: *mut TEdge,
    pub prev_in_sel: *mut TEdge,
}

impl Default for TEdge {
    fn default() -> Self {
        Self {
            bot: IntPoint::default(),
            curr: IntPoint::default(),
            top: IntPoint::default(),
            dx: 0.0,
            poly_typ: PolyType::Subject,
            side: EdgeSide::Left,
            wind_delta: 0,
            wind_cnt: 0,
            wind_cnt2: 0,
            out_idx: 0,
            next: ptr::null_mut(),
            prev: ptr::null_mut(),
            next_in_lml: ptr::null_mut(),
            next_in_ael: ptr::null_mut(),
            prev_in_ael: ptr::null_mut(),
            next_in_sel: ptr::null_mut(),
            prev_in_sel: ptr::null_mut(),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct IntersectNode {
    pub edge1: *mut TEdge,
    pub edge2: *mut TEdge,
    pub pt: IntPoint,
}

impl Default for IntersectNode {
    fn default() -> Self {
        Self {
            edge1: ptr::null_mut(),
            edge2: ptr::null_mut(),
            pt: IntPoint::default(),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct LocalMinimum {
    pub y: CInt,
    pub left_bound: *mut TEdge,
    pub right_bound: *mut TEdge,
}

impl Default for LocalMinimum {
    fn default() -> Self {
        Self {
            y: 0,
            left_bound: ptr::null_mut(),
            right_bound: ptr::null_mut(),
        }
    }
}

#[derive(Debug)]
pub struct OutRec {
    pub idx: i32,
    pub is_hole: bool,
    pub is_open: bool,
    pub first_left: *mut OutRec,
    pub poly_nd: *mut PolyNode,
    pub pts: *mut OutPt,
    pub bottom_pt: *mut OutPt,
}

impl Default for OutRec {
    fn default() -> Self {
        Self {
            idx: 0,
            is_hole: false,
            is_open: false,
            first_left: ptr::null_mut(),
            poly_nd: ptr::null_mut(),
            pts: ptr::null_mut(),
            bottom_pt: ptr::null_mut(),
        }
    }
}

#[derive(Debug)]
pub struct OutPt {
    pub idx: i32,
    pub pt: IntPoint,
    pub next: *mut OutPt,
    pub prev: *mut OutPt,
}

impl Default for OutPt {
    fn default() -> Self {
        Self {
            idx: 0,
            pt: IntPoint::default(),
            next: ptr::null_mut(),
            prev: ptr::null_mut(),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Join {
    pub out_pt1: *mut OutPt,
    pub out_pt2: *mut OutPt,
    pub off_pt: IntPoint,
}

impl Default for Join {
    fn default() -> Self {
        Self {
            out_pt1: ptr::null_mut(),
            out_pt2: ptr::null_mut(),
            off_pt: IntPoint::default(),
        }
    }
}

#[derive(Debug)]
pub struct PolyNode {
    pub(crate) contour: Path,
    pub(crate) childs: Vec<*mut PolyNode>,
    pub(crate) parent: *mut PolyNode,
    pub(crate) index: usize,
    pub(crate) is_open: bool,
    pub(crate) jointype: JoinType,
    pub(crate) endtype: EndType,
}

impl Default for PolyNode {
    fn default() -> Self {
        Self {
            contour: Vec::new(),
            childs: Vec::new(),
            parent: ptr::null_mut(),
            index: 0,
            is_open: false,
            jointype: JoinType::Square,
            endtype: EndType::ClosedPolygon,
        }
    }
}

impl PolyNode {
    pub fn new() -> Self {
        Self::default()
    }

    // C++: PolyNode::ChildCount
    pub fn child_count(&self) -> usize {
        self.childs.len()
    }

    // C++: PolyNode::IsOpen
    pub fn is_open(&self) -> bool {
        self.is_open
    }

    pub fn contour(&self) -> &[IntPoint] {
        &self.contour
    }

    pub fn children(&self) -> PolyNodeChildren<'_> {
        PolyNodeChildren {
            iter: self.childs.iter(),
        }
    }

    pub fn child(&self, index: usize) -> Option<&PolyNode> {
        self.childs.get(index).map(|child| unsafe { &**child })
    }

    // C++: PolyNode::AddChild
    pub(crate) unsafe fn add_child(&mut self, child: *mut PolyNode) {
        let cnt = self.childs.len();
        self.childs.push(child);
        // SAFETY: caller provides a valid child pointer owned by the surrounding PolyTree.
        unsafe {
            (*child).parent = self;
            (*child).index = cnt;
        }
    }

    // C++: PolyNode::GetNext
    pub(crate) fn get_next(&self) -> *mut PolyNode {
        if !self.childs.is_empty() {
            self.childs[0]
        } else {
            self.get_next_sibling_up()
        }
    }

    // C++: PolyNode::GetNextSiblingUp
    pub(crate) fn get_next_sibling_up(&self) -> *mut PolyNode {
        if self.parent.is_null() {
            ptr::null_mut()
        } else {
            // SAFETY: parent is set by add_child from a valid PolyNode pointer.
            unsafe {
                let parent = &*self.parent;
                if self.index == parent.childs.len() - 1 {
                    parent.get_next_sibling_up()
                } else {
                    parent.childs[self.index + 1]
                }
            }
        }
    }

    // C++: PolyNode::IsHole
    pub fn is_hole(&self) -> bool {
        let mut result = true;
        let mut node = self.parent;
        while !node.is_null() {
            result = !result;
            // SAFETY: parent links are maintained by add_child.
            unsafe {
                node = (*node).parent;
            }
        }
        result
    }
}

pub struct PolyNodeChildren<'a> {
    iter: std::slice::Iter<'a, *mut PolyNode>,
}

impl<'a> Iterator for PolyNodeChildren<'a> {
    type Item = &'a PolyNode;

    fn next(&mut self) -> Option<Self::Item> {
        self.iter.next().map(|child| unsafe { &**child })
    }
}

#[derive(Debug, Default)]
pub struct PolyTree {
    pub(crate) node: PolyNode,
    pub(crate) all_nodes: Vec<*mut PolyNode>,
}

impl PolyTree {
    pub fn new() -> Self {
        Self::default()
    }

    // C++: PolyTree::Clear
    pub fn clear(&mut self) {
        for node in self.all_nodes.drain(..) {
            if !node.is_null() {
                // SAFETY: all_nodes owns pointers inserted with Box::into_raw by this port.
                unsafe {
                    drop(Box::from_raw(node));
                }
            }
        }
        self.node.childs.clear();
    }

    // C++: PolyTree::GetFirst
    pub fn first(&self) -> Option<&PolyNode> {
        self.node.child(0)
    }

    pub fn children(&self) -> PolyNodeChildren<'_> {
        self.node.children()
    }

    pub(crate) fn get_first(&self) -> *mut PolyNode {
        self.node.childs.first().copied().unwrap_or(ptr::null_mut())
    }

    // C++: PolyTree::Total
    pub fn total(&self) -> usize {
        let mut result = self.all_nodes.len();
        if result > 0 && self.node.childs[0] != self.all_nodes[0] {
            result -= 1;
        }
        result
    }
}

impl Drop for PolyTree {
    fn drop(&mut self) {
        self.clear();
    }
}
