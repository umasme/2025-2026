use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;

// Must match the value in main.rs
const CONFIDENCE_THRESHOLD: u8 = 40;

// =====================================================================
// ROVER CLEARANCE RADIUS — Dynamic passability check in A*
// =====================================================================
// Instead of relying solely on obstacle inflation to keep the rover safe,
// A* checks a square footprint around each candidate cell. This prevents
// the planner from routing through gaps that are physically too narrow
// for the rover, even if individual cells appear clear.
//
// The TOTAL clearance from an obstacle center to the rover center is:
//   INFLATION_RADIUS_CELLS (in main.rs) + ROVER_CLEARANCE_CELLS
//
// With INFLATION_RADIUS_CELLS = 4 (0.20m) and ROVER_CLEARANCE_CELLS = 5 (0.25m),
// total = 9 cells = 0.45m ≈ rover half-width (0.445m).
//
// Why split into two values instead of just inflating by 9?
//   - Inflation of 9 cells causes nearby obstacles to FUSE into one giant
//     blob on the grid, making valid paths disappear entirely.
//   - Keeping inflation small (4) means obstacles stay distinct on the grid.
//   - The clearance check at query time means A* won't route between them
//     if the gap is too narrow, but WILL route between them if there's room.
// =====================================================================
const ROVER_CLEARANCE_CELLS: isize = 5;

#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
pub enum Direction {
    North, East, South, West, None
}

#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    cost: usize, 
    g: usize,    
    x: isize,
    z: isize,
    dir: Direction,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost) 
    }
}
impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Check if a cell is passable for the rover's full footprint.
/// Returns true if ANY cell within ROVER_CLEARANCE_CELLS of (cx, cz)
/// is above the confidence threshold (i.e. blocked).
#[inline]
fn cell_blocked_for_rover(map: &[[u8; 300]; 300], cx: isize, cz: isize) -> bool {
    for dx in -ROVER_CLEARANCE_CELLS..=ROVER_CLEARANCE_CELLS {
        for dz in -ROVER_CLEARANCE_CELLS..=ROVER_CLEARANCE_CELLS {
            let nx = cx + dx;
            let nz = cz + dz;
            if nx < 0 || nx >= 300 || nz < 0 || nz >= 300 {
                return true; // Out of bounds = blocked
            }
            if map[nx as usize][nz as usize] >= CONFIDENCE_THRESHOLD {
                return true;
            }
        }
    }
    false
}

pub fn find_manhattan_path(
    map: &[[u8; 300]; 300],
    start_x: isize, start_z: isize,
    goal_x: isize, goal_z: isize,
) -> Option<Vec<(isize, isize)>> {
    let mut heap = BinaryHeap::new();

    let mut came_from: HashMap<(isize, isize, Direction), (isize, isize, Direction)> = HashMap::new();
    let mut g_score: HashMap<(isize, isize, Direction), usize> = HashMap::new();

    heap.push(State { cost: 0, g: 0, x: start_x, z: start_z, dir: Direction::None });
    g_score.insert((start_x, start_z, Direction::None), 0);

    let turn_penalty = 15; 

    while let Some(State { cost: _, g, x, z, dir }) = heap.pop() {
        if x == goal_x && z == goal_z {
            let mut path = Vec::new();
            let mut current = (x, z, dir);
            
            // Trace back using the full Directional state
            while current.0 != start_x || current.1 != start_z {
                path.push((current.0, current.1));
                current = came_from[&current];
            }
            path.push((start_x, start_z));
            path.reverse();
            
            return Some(simplify_path(path));
        }

        // FIX 3: Skip stale states in the heap. If we already found a cheaper path 
        // to this exact node+direction, ignore this popped state.
        let current_g = *g_score.get(&(x, z, dir)).unwrap_or(&usize::MAX);
        if g > current_g { continue; }

        let neighbors = [
            (x, z + 1, Direction::North),
            (x + 1, z, Direction::East),
            (x, z - 1, Direction::South),
            (x - 1, z, Direction::West),
        ];

        for (nx, nz, ndir) in neighbors.iter() {
            if *nx < 0 || *nx >= 300 || *nz < 0 || *nz >= 300 { continue; }
            // CHANGED: Check the full rover footprint, not just the single cell.
            // This prevents A* from routing through gaps narrower than the rover.
            if cell_blocked_for_rover(map, *nx, *nz) { continue; }

            let move_cost = if dir == Direction::None || dir == *ndir { 1 } else { 1 + turn_penalty };
            let tentative_g = g + move_cost;

            let neighbor_g = g_score.get(&(*nx, *nz, *ndir)).unwrap_or(&usize::MAX);

            if tentative_g < *neighbor_g {
                came_from.insert((*nx, *nz, *ndir), (x, z, dir));
                g_score.insert((*nx, *nz, *ndir), tentative_g);

                let h = (nx.abs_diff(goal_x) + nz.abs_diff(goal_z)) as usize;
                let f = tentative_g + h;
                
                heap.push(State { cost: f, g: tentative_g, x: *nx, z: *nz, dir: *ndir });
            }
        }
    }
    None
}

fn simplify_path(path: Vec<(isize, isize)>) -> Vec<(isize, isize)> {
    if path.len() < 3 { return path; }
    let mut simplified = Vec::new();
    
    simplified.push(path[0]); 
    
    for i in 1..path.len()-1 {
        let prev = path[i-1];
        let curr = path[i];
        let next = path[i+1];
        
        let dir1 = (curr.0 - prev.0, curr.1 - prev.1);
        let dir2 = (next.0 - curr.0, next.1 - curr.1);
        
        if dir1 != dir2 {
            simplified.push(curr); 
        }
    }
    simplified.push(*path.last().unwrap());
    simplified
}