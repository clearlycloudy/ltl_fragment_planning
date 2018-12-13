extern crate chrono;

extern crate ltl_fragment_planning;
use self::chrono::prelude::*;

use self::ltl_fragment_planning::transition_sys as ts;
use self::ltl_fragment_planning::graph;
use self::ltl_fragment_planning::states;

use std::collections::{HashMap,HashSet,BinaryHeap};
use std::cmp::{Ordering,Reverse};


static mut VERBOSE : bool = false;

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

pub fn generate_moving_obstacle_location_constraints( tsys: &ts::TransitionSys, range: ((i32,i32),(i32,i32)) ) -> HashSet<Vec<i32>> {

    let states = generate_permutation( &tsys.s, 0, 4 );

    let mut proposition = HashSet::new();
    for i in states.iter() {
        
        let a : Vec<_> = i.iter().cloned().collect();
        if a[2] >= (range.0).0 && a[2] <= (range.1).0 &&
           a[3] >= (range.0).1 && a[3] <= (range.1).1 {
            proposition.insert(i.clone());
        }
    }
    
    proposition
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
            !( (a[0]-a[2]).abs() + (a[1]-a[3]).abs() <= 1 ) //give 1 unit margin between moving obstruction and agent
        {
            proposition.insert(i.clone());
        }
    }
    
    proposition
}

pub fn generate_persistence_constraints( tsys: &ts::TransitionSys, persistence_range: ((i32,i32), (i32,i32)) ) -> HashSet<Vec<i32>> {

    let states = generate_permutation( &tsys.s, 0, 4 );
    
    let mut proposition = HashSet::new();
    for i in states.iter() {
        let a : Vec<_> = i.iter().cloned().collect();
        let mut agent_pos = vec![];
        agent_pos.extend_from_slice( &a[0..2] );

        if agent_pos[0] >= (persistence_range.0).0 && agent_pos[0] <= (persistence_range.1).0 &&
           agent_pos[1] >= (persistence_range.0).1 && agent_pos[1] <= (persistence_range.1).1 {
               proposition.insert(i.clone());
           }
    }
    
    proposition
}

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

pub fn generate_task_states( tsys: &ts::TransitionSys, task_pos: &Vec<Vec<i32>> ) -> Vec<HashSet<Vec<i32>>> {
    let states = generate_permutation( &tsys.s, 0, 4 );

    let mut map_task_id = HashMap::new();
    for (k,v) in task_pos.iter().enumerate() {
        map_task_id.insert(v.clone(),k);
    }

    let mut task_pos_hs : HashSet<_> = task_pos.iter().cloned().collect();
        
    let mut proposition = HashMap::new();
    
    for i in states.iter() {
        let a : Vec<_> = i.iter().cloned().collect();
        let mut agent_pos = vec![];
        agent_pos.extend_from_slice( &a[0..2] );
        if task_pos_hs.contains( &agent_pos ) {
            let id = map_task_id.get( &agent_pos ).unwrap();
            if !proposition.contains_key( id ) {
                proposition.insert( *id, HashSet::new() );
            }
            proposition.get_mut(id).unwrap().insert(i.clone());
        }
    }
    
    proposition.values().cloned().collect()
}

fn determine_transition_action( from: &Vec<i32>, to: &Vec<i32> ) -> graph::Action {
    assert!(from.len() == to.len());
    let dy = to[0] - from[0];
    let dx = to[1] - from[1];
    assert!( dx.abs() + dy.abs() == 1 );
    if dy == 1 {
        graph::Action::S
    } else if dy == -1 {
        graph::Action::N
    } else if dx == 1 {
        graph::Action::E
    } else if dx == -1 {
        graph::Action::W
    } else {
        panic!("unexpected delta");
    }
}

fn prune_transition_actions( t: &ts::TransitionSys, dest_states_not_allowed: &HashSet<Vec<i32>> ) -> ts::TransitionSys {

    let mut tsys_filtered : ts::TransitionSys = t.clone();
    
    let mut parent_action_to_prune = HashSet::new();

    for i in dest_states_not_allowed.iter() {
        match tsys_filtered.g.e_reverse.get_mut( i ) { //get parents of state i
            Some( x ) => {
                for j in x.iter() {
                    let a = determine_transition_action( j, i ); //determine what action is taken in parent state
                    parent_action_to_prune.insert( (j.clone(),a) ); //mark (parent state, action) pair to be removed
                }
            },
            _ => {},
        }
    }

    //println!("prunelist: {:?}", parent_action_to_prune );
    
    //prune marked transitions
    for (prune_state, action) in parent_action_to_prune.iter() {
        let states_transition = t.g.get_state_children_from_action( prune_state, *action );
        tsys_filtered.g.prune_destination_transition_for_one( prune_state, &states_transition );
    }

    tsys_filtered.g.update_reverse_edge();

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

fn value_func( tsys: &ts::TransitionSys, dest: &HashSet<Vec<i32>>, states: &Vec<Vec<i32>>, predcessor_type: PredecessorType ) -> ( HashMap<Vec<i32>,i32>, HashMap<Vec<i32>,graph::Action> ) {

    let mut v = HashMap::new(); //storage for computed cost
    let mut policy = HashMap::new(); //storage for policy
    
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
                is_alive.remove( &top.s );
                continue;
            }

            for p in tsys.g.get_state_parents(&top.s).iter() {
                
                if is_alive.contains( p ){


                    let actions = [ graph::Action::N, graph::Action::S, graph::Action::W, graph::Action::E ];

                    let mut actions_val = vec![];
                    
                    for a in actions.iter() {

                        let ( mut t, mut value_action_update) = ( None, None );

                        //get max for all children corresponding to each action
                        for reachable in tsys.g.get_state_children_from_action( p, *a ).iter() {

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
                                value_action_update = Some((temp,a.clone()));
                            } else {
                                if (value_action_update.unwrap()).0 < temp {
                                    t = Some(reachable);
                                    value_action_update = Some((temp,a.clone()));
                                }
                            }
                        }

                        match value_action_update {
                            Some(x) => {
                                actions_val.push( x );
                            },
                            _ => {},
                        }
                    }

                    let best_val_action = match predcessor_type {
                        PredecessorType::Controllable => {
                            //take the min of the values corresponding to different actions
                            actions_val.iter().min_by_key( |x| x.0 )
                        },
                        _ => { //forced predecessor set
                            //take the max of the values corresponding to different actions
                            actions_val.iter().max_by_key( |x| x.0 )
                        },
                    };

                    let v_mut = v.get_mut(p).unwrap();
                    if let Some((val,act)) = best_val_action {
                        if *val < *v_mut {
                            *v_mut = *val;
                            policy.insert( p.clone(), *act );
                            //update the key value of the associated item in heap
                            //by inserting a new copy, ignore old copy in heap if it doesn't match
                            //value in v map

                            //if p[0]==1 && p[1]==1 && p[2] == 2 && p[3] == 2 {
                            //unsafe {
                            // if VERBOSE {
                            //     println!("::::::::::::: updated key value for {:?}->{:?} to {}, action:{:?}",p, top.s, *val, act);
                            // }
                            //}

                            let item = HeapItem { cost: *val, s: p.clone() };
                            q.push( item );
                        }
                    }
                }
            }
            is_alive.remove( &top.s );
        }
    }
    
    (v,policy)
}
    
pub fn ControllablePredecessor( tsys: &ts::TransitionSys, dest: &HashSet<Vec<i32>> ) -> (HashMap<Vec<i32>,i32>, HashMap<Vec<i32>,graph::Action>) {
    
    let states = generate_permutation( &tsys.s, 0, 4 );
    let (costs, policies) = value_func( tsys, dest, &states, PredecessorType::Controllable );
    ( costs.into_iter().filter(|x| x.1 != std::i32::MAX ).collect(), policies )
}

pub fn ForcedPredecessor( tsys: &ts::TransitionSys, dest: &HashSet<Vec<i32>> ) -> (HashMap<Vec<i32>,i32>, HashMap<Vec<i32>,graph::Action>) {
    
    let states = generate_permutation( &tsys.s, 0, 4 );
    let (costs, policies) = value_func( tsys, dest, &states, PredecessorType::Forced );
    ( costs.into_iter().filter(|x| x.1 != std::i32::MAX ).collect(), policies )
}

pub fn OptimalPolicy( tsys: &ts::TransitionSys, dest: &HashSet<Vec<i32>> ) -> HashMap<Vec<i32>,graph::Action> {
    let (costs,policies) = ControllablePredecessor( tsys, dest );
    //println!("len costs: {}, len policies: {}",costs.len(), policies.len());
    policies
}

fn main() {

    let t0 = Local::now();
    
    //dimension of 2D grid: d x d
    let d = 10i32;
    
    // stationary obstacles ---------------------
    let mut stationary_obstacles = HashSet::new();
    stationary_obstacles.insert( vec![1,0] );
    stationary_obstacles.insert( vec![1,1] );
    stationary_obstacles.insert( vec![1,2] );
    stationary_obstacles.insert( vec![1,3] );
    stationary_obstacles.insert( vec![1,4] );
    stationary_obstacles.insert( vec![1,6] );
    stationary_obstacles.insert( vec![1,7] );
    stationary_obstacles.insert( vec![1,8] );
    stationary_obstacles.insert( vec![1,9] );

    // moving obstacle range
    let mv_obs_range = ((6,0),(7,7));
    
    let delete_self_transition_1 = true;
    let (s_map_1, g_map_1) = graph::Graph::build_2d_map( d, &stationary_obstacles, vec![((0,0),(d-1,d-1))], delete_self_transition_1 );
    let delete_self_transition_2 = false;
    let (s_map_2, g_map_2) = graph::Graph::build_2d_map( d, &stationary_obstacles, vec![mv_obs_range.clone()], delete_self_transition_2 );
    
    //state space: agent_position x obstacle_position:
    let (mut s, mut g) = graph::Graph::product_space( (s_map_1,g_map_1), (s_map_2,g_map_2) );

    g.update_reverse_edge();

    let tsys_orig = ts::TransitionSys { s: s, g: g };
    // tsys_orig.query_transitions_possible_partial( &vec![ 0, 0 ] );
    
    println!("resp---------------------------");
        
    //add propositions for next step response-----
    let mut next_step_response : HashMap<(i32,i32), HashSet<(i32,i32)> > = HashMap::new();
    // next_step_response.insert( (3,0), [(3,1)].iter().cloned().collect() );
    
    let next_step_response_allowed = generate_next_step_response_constraints( &tsys_orig, &next_step_response );

    //augment trasition system with next step response constraint----
    let tsys_resp = filter_transition_response( &tsys_orig, &next_step_response_allowed );

    //add propositions for safety------
    let states_satisfy_safety = generate_safety_constraints(&tsys_resp, &stationary_obstacles );

    println!("safe---------------------------");
    
    let tsys_safe = {
        let states : HashSet<Vec<i32>> = generate_permutation( &tsys_resp.s, 0, 4 ).iter().cloned().collect();

        let states_possible = set_intersect( &states, &generate_moving_obstacle_location_constraints( &tsys_resp, mv_obs_range ) );
            
        let states_unsafe = set_subtract( &states_possible, &states_satisfy_safety );

        //generate predecessor states that leads to unsafe states
        let states_unsafe_predecessor = ForcedPredecessor( &tsys_resp, &states_unsafe ).0;

        let keys = states_unsafe_predecessor.keys().cloned().collect::<HashSet<_>>();

        prune_transition_actions( &tsys_resp, &keys )
    };

    println!("persistence---------------------------");
    
    //add propositions for persistence----------------
    let tsys_per = {

        let agent_working_area = ((1,0),(d,d));
        let states_satisfy_persistence = generate_persistence_constraints(&tsys_safe, agent_working_area);

        let states : HashSet<Vec<i32>> = generate_permutation( &tsys_safe.s, 0, 4 ).iter().cloned().collect();

        let states_possible = set_intersect( &states, &generate_moving_obstacle_location_constraints( &tsys_safe, mv_obs_range ) );
        
        let states_persistence_negation = set_subtract( &states_possible, &states_satisfy_persistence );

        //generate predecessor states that leads to non persistent satisfying states
        let states_unpersistence_predecessor = ForcedPredecessor( &tsys_safe, &states_persistence_negation ).0;

        let keys = states_unpersistence_predecessor.keys().cloned().collect::<HashSet<_>>(); 

        prune_transition_actions( &tsys_safe, &keys )
    };

    //add propositions for steady state response------

    println!("ss_resp---------------------------");

    let mut ss_next_step_response : HashMap<(i32,i32), HashSet<(i32,i32)> > = HashMap::new();
    // ss_next_step_response.insert( (4,2), [(5,2)].iter().cloned().collect() );

    let ss_next_step_response_allowed = generate_next_step_response_constraints( &tsys_per, &ss_next_step_response );

    let tsys_ss_resp = filter_transition_response( &tsys_per, &ss_next_step_response_allowed );
    // tsys_ss_resp.query_transitions_possible_partial( &vec![ 1, 2 ] );

    println!("task---------------------------");
    
    //todo: collect states for each task group and compute winning set
        
    // let mut task_pos = HashSet::new();
    let mut task_pos = vec![];
    task_pos.push( vec![9,1] );
    task_pos.push( vec![d-1,d-1] );
    task_pos.push( vec![4,8] );
    task_pos.push( vec![2,0] );
    task_pos.push( vec![2,9] );
    task_pos.push( vec![4,2] );
    let mut task_sets = generate_task_states(&tsys_ss_resp, &task_pos );

    for i in task_sets.iter_mut() {
        let states : HashSet<Vec<i32>> = generate_permutation( &tsys_resp.s, 0, 4 ).iter().cloned().collect();

        let states_possible = set_intersect( &states, &generate_moving_obstacle_location_constraints( &tsys_resp, mv_obs_range ) );
        
        *i = set_intersect( &i, &states_possible );
    }

    let (w, task_groups) = compute_winning_set( &tsys_ss_resp, &task_sets );

    let predecessor_to_winning_set : HashSet<_>= ControllablePredecessor( &tsys_safe, &w ).0.keys().cloned().collect();
    
    println!("winning set length: {:?}", w.len() );
    println!("predecessor_to_winning_set length: {:?}", predecessor_to_winning_set.len() );
    // println!("sizes of task sets:");
    // task_groups.iter().for_each( |x| { println!("{:?}",x.len());} );
    // println!("{:?}", tsys_resp.query_transitions_possible_partial( &vec![0,0] ) );
    
    let initial_state = vec![0,0,(mv_obs_range.1).0, (mv_obs_range.1).1];
    if !predecessor_to_winning_set.contains(&initial_state) {
        println!("infeasible");

        println!("winning set length: {:?}", w.len() );
        println!("predecessor_to_winning_set length: {:?}", predecessor_to_winning_set.len() );

        let t1 = Local::now();
        let t_delta = t1.signed_duration_since(t0).num_microseconds().unwrap() as f64;
        println!( "synthesis time duration: {}ms", t_delta /1000.0);
        
    } else {
        println!("feasible");
        
        //generate feasible policy from s0 to winning set
        unsafe{ VERBOSE = true; }
        let policy_to_w = OptimalPolicy( &tsys_safe, &w );

        unsafe{ VERBOSE = false; }
        //generate feasible policy for each task set
        let policy_to_tasks : Vec<_>= task_groups.iter().map(|x| OptimalPolicy( &tsys_safe, x ) ).collect();

        let t1 = Local::now();
        let t_delta = t1.signed_duration_since(t0).num_microseconds().unwrap() as f64;
        println!( "synthesis time duration: {}ms", t_delta /1000.0);
        
        //construct task graph to task set optimizer, currently not implemented
        // let t_graph = generate_task_graph( &tsys_ss_resp, &task_groups );
        //optimal task set traversal policy can be inserted here (eg: minimizing bottleneck cost of traversal between tasks), but currently not implemented

        //run simulation
        println!("start simulation");
        let mut sim_state = vec![ 0, 0, (mv_obs_range.1).0, (mv_obs_range.1).1 ];

        let mut task_visit_count = HashMap::new();
        let mut task_pos_order = vec![];
        for i in task_pos.iter() {
            task_visit_count.insert(i.clone(),0);
            task_pos_order.push( i.clone() );
        }
        let mut visit_count = 0;
        //stopping condition of simulation
        let visit_count_lim = policy_to_tasks.len() * 3;
        let mut task_cur = 0;

        let mut run_state = vec![ (sim_state.clone(), None, None, false) ];

        //run policy from s0 to winning set and then switch to policies for task sets
        let mut reached_winning_set = false;

        println!("using policy to reach winning set");
        
        'next_step: loop {

            let mut reached_task_set = false;
            
            if visit_count >= visit_count_lim {
                break;
            }

            // println!("sim_state: {:?}", sim_state );
            let mut pos_agent : Vec<_> = sim_state.iter().take(2).cloned().collect();

            let next_action = if !reached_winning_set {
                match policy_to_w.get(&sim_state) {
                    Some(x) => { x },
                    None => {
                        if w.contains(&sim_state) {
                            reached_winning_set = true;
                            // println!("reached winning set, switching policy to reach tasks");
                            continue 'next_step;
                        } else {
                            panic!("unexpected state in policy query to reach winning set");
                        }
                    }
                }
            } else {
                match policy_to_tasks[task_cur].get(&sim_state) {
                    Some(x) => { x },
                    None => {
                        if task_pos.contains(&pos_agent) {
                            reached_task_set = true;
                            // println!("reached task destination: {:?}", pos_agent );
                            task_cur = (task_cur + 1) % policy_to_tasks.len();
                            visit_count += 1;
                            policy_to_tasks[task_cur].get(&sim_state).expect("unexpected state in policy query")
                        } else {
                            panic!("unexpected state in policy query: {:?}", sim_state);
                        }
                    }
                }
            };

            match next_action {
                graph::Action::N => { pos_agent[0] -= 1; },
                graph::Action::S => { pos_agent[0] += 1; },
                graph::Action::W => { pos_agent[1] -= 1; },
                graph::Action::E => { pos_agent[1] += 1; },
                graph::Action::Stationary => {},
            }

            // println!("agent pos new: {:?}", pos_agent );
            let mut pos_mov_obs : Vec<_> = sim_state.iter().skip(2).cloned().collect();
            use rand::distributions::{Distribution,Uniform};
            let mut rng = rand::thread_rng();
            let btw = Uniform::from(0..4);
            let moving_obs_action = loop {
                let choice = btw.sample(&mut rng);
                let choices = [ graph::Action::N, graph::Action::S, graph::Action::W, graph::Action::E ];
                let chosen = match choices[choice] {
                    graph::Action::N if pos_mov_obs[0] - 1 >= (mv_obs_range.0).0 && pos_mov_obs[0] - 1 <= (mv_obs_range.1).0 => {
                        Some(graph::Action::N)
                    },
                    graph::Action::S if pos_mov_obs[0] + 1 >= (mv_obs_range.0).0 && pos_mov_obs[0] + 1 <= (mv_obs_range.1).0 => {
                        Some(graph::Action::S)
                    },
                    graph::Action::W if pos_mov_obs[1] - 1 >= (mv_obs_range.0).1 && pos_mov_obs[1] - 1 <= (mv_obs_range.1).1 => {
                        Some(graph::Action::W)
                    },
                    graph::Action::E if pos_mov_obs[1] + 1 >= (mv_obs_range.0).1 && pos_mov_obs[1] + 1 <= (mv_obs_range.1).1 => {
                        Some(graph::Action::E)
                    },
                    _ => None
                };
                match chosen {
                    Some(x) => {
                        break x
                    },
                    _ => {continue;},
                }
            };
            
            match moving_obs_action {
                graph::Action::N => { pos_mov_obs[0] -= 1; },
                graph::Action::S => { pos_mov_obs[0] += 1; },
                graph::Action::W => { pos_mov_obs[1] -= 1; },
                graph::Action::E => { pos_mov_obs[1] += 1; },
                graph::Action::Stationary => {},
            }
            // println!("agent_pos new, moving obs pos new: {:?}, {:?}", pos_agent, pos_mov_obs );

            sim_state[0] = pos_agent[0];
            sim_state[1] = pos_agent[1];
            sim_state[2] = pos_mov_obs[0];
            sim_state[3] = pos_mov_obs[1];

            //verify that moving obstruction does not come close to the agent
            if (sim_state[0]-sim_state[2]).abs() + (sim_state[1]-sim_state[3]).abs() <= 1 {
                println!("agent came too close to the moving obstruction");
                break;
            }
            
            run_state.push( ( sim_state.clone(), Some(next_action), Some(moving_obs_action), reached_task_set ) );
        }

        // println!("run: {:?}", run_state );

        //log run to file
        use std::fmt;
        use std::fs::File;
        use std::io::prelude::*;
        use std::io::Write;
        let mut buf = String::new();
        let mut file = File::create("sample_task_set_6.txt").expect("log file creation");
        for (idx,(i,j,k,l)) in run_state.iter().enumerate() {
            fmt::write(& mut buf,format_args!("{}, {}, {}, {}, {}, {:?}, {:?}, {}\n", idx, i[0], i[1], i[2], i[3], j, k, l )).expect("log string write");
        }
        file.write_all(buf.as_bytes());
        
    }
    
    return
}

pub fn set_intersect( a: &HashSet< Vec<i32> >, b: &HashSet< Vec<i32> > ) -> HashSet< Vec<i32> > {
    let mut ret = HashSet::new();
    for i in a.iter() {
        if b.contains( i ) {
            ret.insert(i.clone());
        }
    }
    ret
}


fn compute_winning_set( tsys: &ts::TransitionSys, task_sets: &Vec<HashSet<Vec<i32>>> ) -> ( HashSet<Vec<i32>>, Vec<HashSet<Vec<i32>>> ) {
    let count_tasks = task_sets.len();
    let mut feasible_sets = task_sets.clone();
    let mut w = HashSet::new();
    loop {
        for (idx,task_set) in task_sets.iter().take(count_tasks-1).enumerate() {
            let predecessor_set = ControllablePredecessor( &tsys, task_set ).0.keys().cloned().collect();
            feasible_sets[idx+1] = set_intersect( &feasible_sets[idx+1], &predecessor_set );
            if feasible_sets[idx+1].len() == 0 {
                return ( HashSet::new(), vec![] )
            }
        }
        let predecessor_set = ControllablePredecessor( &tsys, &feasible_sets[count_tasks-1] ).0.keys().cloned().collect();
        
        if set_subtract( &feasible_sets[0], &predecessor_set ).len() == 0 {
            //test for f_0 subset f_{n-1}
            w = predecessor_set;
            break;
        }
        feasible_sets[0] = set_intersect( &feasible_sets[0], &predecessor_set );
    }
    ( w, feasible_sets )
}

fn generate_task_graph( tsys: &ts::TransitionSys, task_sets: &Vec<HashSet<Vec<i32>>> ){
    
    unimplemented!();
}
