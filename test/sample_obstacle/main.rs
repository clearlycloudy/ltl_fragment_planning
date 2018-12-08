extern crate ltl_fragment_planning;

use self::ltl_fragment_planning::transition_sys as ts;
use self::ltl_fragment_planning::graph;
use self::ltl_fragment_planning::states;

use std::collections::{HashMap,HashSet,BinaryHeap};
use std::cmp::{Ordering,Reverse};


pub fn generate_permutation( s:&states::States, index_start: usize, count: usize ) -> Vec< Vec< i32 > > {
    let mut a :Vec<Vec<i32>> = vec![vec![]];
    for i in s.s_dim.iter().skip(index_start).take(count) {
        let mut a_final = vec![];
        for j in 0..*i {
            let mut a_copy = a.clone();
            for k in a_copy.iter_mut() {
                k.push(j);
            }
            for k in a_copy {
                a_final.push(k);
            }
        }
        a = a_final;
    }
    a
}

pub fn set_subtract( a: &HashSet< Vec<i32> >, b: &HashSet< Vec<i32> > ) -> HashSet< Vec<i32> > {
    let mut ret = HashSet::new();
    for i in a.iter() {
        if !b.contains( i ) {
            ret.insert(i.clone());
        }
    }
    ret
}

//generates states that are feasible in terms of safety
pub fn generate_safety_constraints( tsys: &ts::TransitionSys, obstacles_stationary: &HashSet<Vec<i32>> ) -> HashSet<Vec<i32>> {

    let states = generate_permutation( &tsys.s, 0, 4 );

    let mut proposition = HashSet::new();
    for i in states.iter() {
        
        let a : Vec<_> = i.iter().cloned().collect();
        let mut agent_pos = vec![];
        agent_pos.extend_from_slice( &a[0..2] );
        if !obstacles_stationary.contains( &agent_pos ) &&
            !(a[0] >= a[2]-1 && a[0] <= a[2]+1 && //give a 1 unit length of margin for moving obstacle
              a[1] >= a[3]-1 && a[1] <= a[3]+1) {
            // !(a[0] == a[2] &&
            //   a[1] == a[3]) {
            proposition.insert(i.clone());
        }
    }
    
    proposition
}

// pub fn generate_persistence_constraints( s: &States, persistence_range: ((i32,i32), (i32,i32)) ) -> HashSet<Vec<i32>> {

//     let states = generate_permutation( &s, 0, 4 );
    
//     let mut proposition = HashSet::new();
//     for i in states.iter() {
//         let a : Vec<_> = i.iter().cloned().collect();
//         let mut agent_pos = vec![];
//         agent_pos.extend_from_slice( &a[0..2] );

//         if agent_pos[0] >= (persistence_range.0).0 && agent_pos[0] <= (persistence_range.1).0 &&
//            agent_pos[1] >= (persistence_range.0).1 && agent_pos[1] <= (persistence_range.1).1 {
//                proposition.insert(i.clone());
//            }
//     }
    
//     proposition
// }

pub fn generate_next_step_response_constraints( tsys: &ts::TransitionSys, next_step_response: &HashMap<(i32,i32),HashSet<(i32,i32)>> ) -> HashMap<Vec<i32>, HashSet< Vec<i32> > > {
    
    let states = generate_permutation( &tsys.s, 0, 4 );
    
    let mut agent_pos_to_state : HashMap<(i32,i32),HashSet<Vec<i32>>> = HashMap::new();

    for i in states.iter() {
        if !agent_pos_to_state.contains_key( &(i[0],i[1]) ) {
            agent_pos_to_state.insert( (i[0],i[1]), HashSet::new() );
        }
        agent_pos_to_state.get_mut(&(i[0],i[1])).unwrap().insert( i.clone() );
    }

    let mut proposition = HashMap::new();
    
    for (k,v) in next_step_response.iter() {
        let agent_pos = k;
        let mut source = vec![];
        match agent_pos_to_state.get( &agent_pos ) {
            Some(x) => {
                for y in x.iter(){
                    source.push(y.clone());
                }
            },
            _ => {},
        }
        for agent_next_pos in v.iter() {
            let mut dest = vec![];
            match agent_pos_to_state.get( agent_next_pos ) {
                Some(x) => {
                    for y in x.iter(){
                        dest.push(y.clone());
                    }
                },
                _ => {},
            }
            for src in source.iter() {
                if !proposition.contains_key( src ) {
                    proposition.insert( src.clone(), HashSet::new() );
                }
                for dst in dest.iter() {
                    proposition.get_mut(src).unwrap().insert( dst.clone() );
                }
            }
        }
    }
    
    proposition
}

// pub fn generate_task_states( s: &States, task_pos: &HashSet<Vec<i32>> ) -> HashMap<usize,HashSet<Vec<i32>>> {
//     let states = generate_permutation( &s, 0, 4 );

//     let mut map_task_id = HashMap::new();
//     for (k,v) in task_pos.iter().enumerate() {
//         map_task_id.insert(v.clone(),k);
//     }

//     let mut proposition = HashMap::new();
    
//     for i in states.iter() {
//         let a : Vec<_> = i.iter().cloned().collect();
//         let mut agent_pos = vec![];
//         agent_pos.extend_from_slice( &a[0..2] );
//         if task_pos.contains( &agent_pos ) {
//             let id = map_task_id.get( &agent_pos ).unwrap();
//             if !proposition.contains_key( id ) {
//                 proposition.insert( *id, HashSet::new() );
//             }
//             proposition.get_mut(id).unwrap().insert(i.clone());
//         }
//     }
    
//     proposition
// }

fn filter_transition_destination( t: &ts::TransitionSys, states_allowed: &HashSet<Vec<i32>> ) -> ts::TransitionSys {

    let mut tsys_filtered = ts::TransitionSys::default();
    
    for (k,v) in t.g.e.iter() {
        if !tsys_filtered.g.e.contains_key( k ) {
            tsys_filtered.g.e.insert( k.clone(), HashSet::new() );
        }

        for j in v.iter() {
            if states_allowed.contains(j) {
                tsys_filtered.g.e.get_mut( k ).unwrap().insert( j.clone() );
            }
        }
    }

    tsys_filtered.g.update_reverse_edge();
    tsys_filtered.s = t.s.clone();

    tsys_filtered
}

fn filter_transition_response( t: &ts::TransitionSys, response_allowed: &HashMap<Vec<i32>, HashSet<Vec<i32>>> ) -> ts::TransitionSys {

    let mut tsys_filtered = ts::TransitionSys::default();
    
    for (k,v) in t.g.e.iter() {

        if !tsys_filtered.g.e.contains_key( k ) {
            tsys_filtered.g.e.insert( k.clone(), HashSet::new() );
        }
        
        match response_allowed.get( k ) {
            Some(next_state_permitted) => {
                for next_state in v.iter() {
                    if next_state_permitted.contains( next_state ) {
                        tsys_filtered.g.e.get_mut( k ).unwrap().insert( next_state.clone() );
                    }
                }
            },
            _ => {
                for next_state in v.iter() {
                    tsys_filtered.g.e.get_mut( k ).unwrap().insert( next_state.clone() );
                }
            },
        }
    }

    tsys_filtered.g.update_reverse_edge();
    tsys_filtered.s = t.s.clone();

    tsys_filtered
}

#[derive(Eq,Debug)]
struct HeapItem {
    cost: i32,
    s: Vec<i32>,
}

impl Ord for HeapItem {
    fn cmp( &self, other: &Self ) -> Ordering {
        match self.cost.cmp( &other.cost ) {
            Ordering::Less => { Ordering::Greater },
            Ordering::Greater => { Ordering::Less },
            _ => Ordering::Equal,
        }
    }
}
impl PartialOrd for HeapItem {
    fn partial_cmp(&self, other: &Self ) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for HeapItem {
    fn eq(&self, other: &Self ) -> bool {
        self.cost == other.cost
    }
}

pub enum PredecessorType {
    Controllable,
    Forced,
}

fn value_func( tsys: &ts::TransitionSys, dest: &HashSet<Vec<i32>>, states: &Vec<Vec<i32>>, predcessor_type: PredecessorType ) -> HashMap<Vec<i32>,i32> {

    let mut v = HashMap::new();
    
    for i in states.iter() {
        v.insert( i.clone(), std::i32::MAX );
    }

    for i in dest.iter() {
        v.insert( i.clone(), 0 );
    }
    
    let mut q : BinaryHeap<HeapItem>= BinaryHeap::new();

    for i in states.iter() {
        let item = HeapItem { cost: *v.get(i).unwrap(), s: i.clone() };
        q.push( item );
    }

    let mut is_alive = HashSet::new();
    for i in states.iter() {
        is_alive.insert( i.clone() );
    }
    
    while !q.is_empty() {
        let top = q.pop().unwrap();
        if top.cost != *v.get( &top.s ).unwrap() {
            // println!("ignored:{:?}",top);
            //must have updated the cost for this entry, so ignore this entry
            continue;
        } else {
            if top.cost == std::i32::MAX {
                // println!("q: {:?}",q);                
                // println!("break: {:?}",top);
                is_alive.remove( &top.s ); //todo remove this once bug fixed
                // break;
                continue;
            }

            //println!("tsys.g : {:?}",tsys.g.e_reverse);
            for p in tsys.g.get_state_parents(&top.s).iter() {
                
                if is_alive.contains( p ){


                    let actions = [ graph::Action::N, graph::Action::S, graph::Action::W, graph::Action::E ];

                    let mut actions_val = vec![];
                    
                    for a in actions.iter() {

                        let (mut t, mut value_update) = (None,None);

                        //get max for all children corresponding to each action
                        for reachable in tsys.g.get_state_children_from_action( p, *a ).iter() {

                            if p[0]==1 && p[1]==1 && p[2] == 2 && p[3] == 2 {
                                // println!("top: {:?}, p: {:?}, action: {:?}, reachable: {:?}", top.s, p, a, tsys.g.get_state_children_from_action( p, *a ) );
                            }
                            //assume unit cost between reachable neighbours, s and t
                            let cost_s_to_t = 1;

                            let val_reachable = *v.get(reachable).unwrap();
                            let temp = if val_reachable != std::i32::MAX {
                                val_reachable + cost_s_to_t
                            } else {
                                std::i32::MAX
                            };
                            if t == None {
                                t = Some(reachable);
                                value_update = Some(temp);
                                // if p[0]==1 && p[1]==1 && p[2] == 2 && p[3] == 2 {
                                //     println!("here: top: {:?}, p: {:?}, action: {:?}, reachable: {:?}", top.s, p, a, tsys.g.get_state_children_from_action( p, *a ) );
                                //     println!("reach: {:?}, value_update: {:?}", reachable, value_update);
                                // }
                            } else {
                                if value_update.unwrap() < temp {
                                    t = Some(reachable);
                                    value_update = Some(temp);
                                    // if p[0]==1 && p[1]==1 && p[2] == 2 && p[3] == 2 {
                                        // println!("top: {:?}, p: {:?}, action: {:?}, reachable: {:?}", top.s, p, a, tsys.g.get_state_children_from_action( p, *a ) );
                                        // println!("{:?}, reach: {:?}, value_update: {:?}", p, reachable, value_update);
                                    // }
                                }
                            }
                        }

                        match value_update {
                            Some(x) => {
                                actions_val.push( x );
                            },
                            _ => {},
                        }
                    }

                    //take the min of the values corresponding to different actions
                    let m = match predcessor_type {
                        PredecessorType::Controllable => {
                            actions_val.iter().min()
                        },
                        _ => {
                            actions_val.iter().max()
                        },
                    };

                    // if p[0]==1 && p[1]==1 && p[2] == 2 && p[3] == 2 {
                    //     println!(">>>>>>.top: {:?}, p: {:?}, actions_val: {:?}", top.s, p, actions_val );
                    // }
                    
                    // if p[0]==0 && p[1]==0 {
                    //     println!("top: {:?}", top.s );
                    //     println!("actions: {:?}", actions_val );
                    // }

                    let v_mut = v.get_mut(p).unwrap();
                    if let Some(val) = m {
                        if *val < *v_mut {
                            *v_mut = *val;
                            //update the key value of the associated item in heap
                            //by inserting a new copy, ignore old copy in heap if it doesn't match
                            //value in v map
                            // if p[0]==1 && p[1]==1 && p[2] == 2 && p[3] == 2 {
                                println!("::::::::::::: updated key value for {:?}->{:?} to {}",p, top.s, *val);
                            // }
                            let item = HeapItem { cost: *val, s: p.clone() };
                            q.push( item );
                        }
                    }
                }
            }
            is_alive.remove( &top.s );
        }
    }
    
    v
}
    
pub fn ControllablePredecessor( tsys: &ts::TransitionSys, dest: &HashSet<Vec<i32>> ) -> HashMap<Vec<i32>,i32> {
    
    let states = generate_permutation( &tsys.s, 0, 4 );
    let mut ret = HashMap::new();
    for (k,v) in value_func( tsys, dest, &states, PredecessorType::Controllable ).iter() {
        if *v != std::i32::MAX {
            ret.insert(k.clone(),v.clone());
        }
    }
    ret
}

pub fn ForcedPredecessor( tsys: &ts::TransitionSys, dest: &HashSet<Vec<i32>> ) -> HashMap<Vec<i32>,i32> {
    
    let states = generate_permutation( &tsys.s, 0, 4 );
    let mut ret = HashMap::new();
    for (k,v) in value_func( tsys, dest, &states, PredecessorType::Forced ).iter() {
        if *v != std::i32::MAX {
            ret.insert(k.clone(),v.clone());
        }
    }
    ret
}

fn main() {
    
    println!( "ltl_fragment_planning, version {}", ts::version() );
    
    // map (3,0) represents south west corner
    // A:= agent, O:=moving obstruction ((1,0),(2,2))
    // X:=stationary obstruction, T:=task
    //
    //   012
    // 0 A T             
    // 1  X  
    // 2 T O

    // safety propositions ---------------------
    // do not collide with O and X
    let mut stationary_obstacles = HashSet::new();
    // stationary_obstacles.insert( vec![1,1] );
    
    // propositions for next step response:
    // for agent: (0,0) -> (0,1)

    // steady state next step response:
    // for agent pos: none

    // persistence proposition:
    // remain in rectangle (0,0) bounded by (3,3)
    
    // task proposition:
    // visit each of T's infinitely often

    //------------------------------------------
        
    //create initial transition system
    let d = 8i32;

    let delete_self_transition_1 = true;
    let (s_map_1, g_map_1) = graph::Graph::build_2d_map( d, &HashSet::new(), ((0,0),(d,d)), delete_self_transition_1 );
    // let (s_map_2, g_map_2) = graph::Graph::build_2d_map( d, &stationary_obstacles, ((1,0),(1,2)) );
    let delete_self_transition_2 = false;
    let (s_map_2, g_map_2) = graph::Graph::build_2d_map( d, &stationary_obstacles, ((0,0),(2,2)), delete_self_transition_2 );
    
    //state space: agent_position x obstacle_position:
    let (mut s, mut g) = graph::Graph::product_space( (s_map_1,g_map_1), (s_map_2,g_map_2) );
    
    g.update_reverse_edge();

    s.s[0] = 0;
    s.s[1] = 0;

    s.s[2] = 0;
    s.s[3] = d-1;

    let tsys_orig = ts::TransitionSys { s: s, g: g };

    //add propositions for next step response-----
    let mut next_step_response : HashMap<(i32,i32), HashSet<(i32,i32)> > = HashMap::new();
    next_step_response.insert( (1,0), [(1,1)].iter().cloned().collect() );
    
    let next_step_response_allowed = generate_next_step_response_constraints( &tsys_orig, &next_step_response );

    //augment trasition system with next step response constraint----
    let tsys_resp = filter_transition_response( &tsys_orig, &next_step_response_allowed );
    // tsys_resp.query_transitions_possible( &vec![ 0, 0, 6, 6] );
    // tsys_resp.query_transitions_possible_partial( &vec![ 0, 0 ] );
    // let dest = tsys_resp.query_states_partial_nonempty( &vec![0,0] );

    //add propositions for safety------
    // let moving_obstacle_range = ((4,1),(8,7));
    let states_satisfy_safety = generate_safety_constraints(&tsys_resp, &stationary_obstacles );
    // println!("{:?}",states_satisfy_safety.len());

    let tsys_safe = {
        let states : HashSet<Vec<i32>> = generate_permutation( &tsys_resp.s, 0, 4 ).iter().cloned().collect();
        let states_unsafe = set_subtract( &states, &states_satisfy_safety );
        // println!("unsafe states: {:?}",states_unsafe);
        
        //generate predecessor states that leads to unsafe states
        let states_unsafe_predecessor = ForcedPredecessor( &tsys_resp, &states_unsafe );
        // println!("states guaranteed to lead to unsafety: {:?}", states_unsafe_predecessor);

        let keys = states_unsafe_predecessor.keys().cloned().collect::<HashSet<_>>();
        let states_remain = set_subtract( &states, &keys );
        // println!("states remain: {:?}", states_remain );

        filter_transition_destination( &tsys_resp, &states_remain )
    };

    //add propositions for persistence----------------
    let agent_working_area = ((d/2,d/2),(d,d));
    // // let states_satisfy_persistence = generate_persistence_constraints(&tsys_orig.s, persistence_range);
    
    // // let tsys_pers = filter_transition_destination( &tsys_safety, &states_satisfy_persistence );
    // // tsys_pers.query_transitions_possible( &vec![ 3, 3, 6, 6] );
    
    return


    // // //------------------------------------------------
    
    // // //add propositions for steady state response------
    // // let mut ss_next_step_response : HashMap<(i32,i32), HashSet<(i32,i32)> > = HashMap::new();
    // // ss_next_step_response.insert( (9,8), [(8,8)].iter().cloned().collect() );
    // // ss_next_step_response.insert( (8,8), [(8,9)].iter().cloned().collect() );

    // // let ss_next_step_response_allowed = generate_next_step_response_constraints( &tsys_orig.s, &ss_next_step_response );

    // // let tsys_ss_resp = filter_transition_response( &tsys_pers, &ss_next_step_response_allowed );
    // // tsys_ss_resp.query_transitions_possible( &vec![ 9, 8, 6, 6] );
    // // tsys_ss_resp.query_transitions_possible( &vec![ 8, 8, 6, 6] );
    // // //------------------------------------------------

    // // //collect all states for task---------------------
    // // let mut task_pos = HashSet::new();
    // // task_pos.insert( vec![3,6] );
    // // task_pos.insert( vec![8,0] );
    // // task_pos.insert( vec![9,9] );
    // // let task_states = generate_task_states(&tsys_orig.s, &task_pos );
    // // println!("task_groups len: {}", task_states.len());
    // // //------------------------------------------------

    // // let (w, task_groups) = compute_winning_set( &tsys_ss_resp, &task_states );
    
}
