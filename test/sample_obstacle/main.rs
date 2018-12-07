extern crate ltl_fragment_planning;

use self::ltl_fragment_planning::transition_sys as ts;

use std::collections::{HashMap,HashSet};

#[derive(Debug,Clone,Copy)]
pub enum AgentAction { //4 canonical directions
    N,
    W,
    S,
    E
}

pub enum AtomicProposition {
    SurveillanceArea,
    NoCollision,
}

#[derive(Default,Debug,Clone)]
pub struct PropositionSafety(Vec<i32>);

#[derive(Default,Debug,Clone)]
pub struct PropositionTasks(Vec<i32>);

#[derive(Default,Debug,Clone)]
pub struct Graph {
    pub e: HashMap<Vec<i32>, HashSet<Vec<i32>> >,
}

impl Graph {
    
    pub fn build_2d_map( d: i32 ) -> ( States, Graph ) {
        
        let mut g = Graph::default();

        let states = States {
            s: vec![ 0, 0 ],
            s_dim: vec![ d, d ]
        };
        
        for i in 0..d {
            for j in 0..d {
                if !g.e.contains_key(&vec![i,j]) {
                    g.e.insert(vec![i,j], HashSet::new() );
                }
                for k in i-1..=i+1 {
                    if k < d && k >= 0 {
                        g.e.get_mut(&vec![i,j]).unwrap().insert( vec![k,j] );
                    }
                }
                for k in j-1..=j+1 {
                    if k < d && k >= 0 {
                        g.e.get_mut(&vec![i,j]).unwrap().insert( vec![i,k] );
                    }
                }
            }
        }

        ( states, g )
    }
}

//just append the state from s2 to the already existing codomain items in g1
pub fn product_space( (s1,g1): (States,Graph), s2: States ) -> ( States, Graph ) {
    let mut s = s1.clone();
    for i in s2.s_dim.iter() {
        s.s_dim.push(*i);
        s.s.push(0); //to be changed later
    }
    let mut g = g1.clone();
    for i in s2.s_dim {
        let mut g_temp = Graph::default();
        for j in 0..i {
            for (k,v) in g.e.iter() {
                let mut k2 = k.clone();
                k2.push(j);
                if !g.e.contains_key(&k2) {
                    g_temp.e.insert( k2.clone(), HashSet::new() );
                }
                for codomain in v.iter() {
                    let mut codomain2 = codomain.clone();
                    codomain2.push(j);
                    g_temp.e.get_mut( &k2 ).unwrap().insert( codomain2 );
                }
            }
        }
        use std::mem;
        mem::swap( & mut g, & mut g_temp );
    }
    ( s, g )
}

#[derive(Debug,Clone,Default)]
pub struct States {
    s: Vec<i32>,
    s_dim: Vec<i32>, //indicates dimension of each state
}

#[derive(Debug,Default)]
pub struct TransitionSys {
    s: States,
    g: Graph,
}

impl TransitionSys {
    pub fn query_transitions_possible( &self, state: &Vec<i32> ) {
        match self.g.e.get( state ) {
            Some(x) => {
                println!("transition for: {:?}", state );
                for i in x {
                    println!("{:?}", i );
                }
            },
            _ => {},
        }
    }
}

pub fn generate_permutation( s:&States, index_start: usize, count: usize ) -> Vec< Vec< i32 > > {
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


pub fn generate_safety_constraints( s: &States, obstacles_stationary: &HashSet<Vec<i32>>, obstacle_moving_range: ((i32,i32), (i32,i32)) ) -> HashSet<Vec<i32>> {

    //generate sample stationary obstacles

    let states = generate_permutation( &s, 0, 4 );
    
    let mut proposition = HashSet::new();
    for i in states.iter() {
        let a : Vec<_> = i.iter().cloned().collect();
        let obstruction_pos = &a[2..4];
        let mut agent_pos = vec![];
        agent_pos.extend_from_slice( &a[0..2] );
        if !obstacles_stationary.contains( &agent_pos )
        {
            //further prune states in which agent can potentially collide with moving obstacle
            
            let dy = agent_pos[0] - obstruction_pos[0];
            let dx = agent_pos[1] - obstruction_pos[1];
            let candidate = if dy.abs() + dx.abs() == 0 {
                false
            } else if (dy.abs() + dx.abs() == 1 ) &&
                agent_pos[0] >= (obstacle_moving_range.0).0 && agent_pos[0] <= (obstacle_moving_range.1).0 &&
                agent_pos[1] >= (obstacle_moving_range.0).1 && agent_pos[1] <= (obstacle_moving_range.1).1 {
                false
            } else {
                true
            };
            if candidate {
                proposition.insert(i.clone());
            }
        }
    }
    
    proposition
}

pub fn generate_persistence_constraints( s: &States, persistence_range: ((i32,i32), (i32,i32)) ) -> HashSet<Vec<i32>> {

    let states = generate_permutation( &s, 0, 4 );
    
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

pub fn generate_next_step_response_constraints( s: &States, next_step_response: &HashMap<(i32,i32),HashSet<(i32,i32)>> ) -> HashMap<Vec<i32>, HashSet< Vec<i32> > > {
    
    let states = generate_permutation( &s, 0, 4 );
    
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

pub fn generate_task_states( s: &States, task_pos: &HashSet<Vec<i32>> ) -> HashMap<usize,HashSet<Vec<i32>>> {
    let states = generate_permutation( &s, 0, 4 );

    let mut map_task_id = HashMap::new();
    for (k,v) in task_pos.iter().enumerate() {
        map_task_id.insert(v.clone(),k);
    }

    let mut proposition = HashMap::new();
    
    for i in states.iter() {
        let a : Vec<_> = i.iter().cloned().collect();
        let mut agent_pos = vec![];
        agent_pos.extend_from_slice( &a[0..2] );
        if task_pos.contains( &agent_pos ) {
            let id = map_task_id.get( &agent_pos ).unwrap();
            if !proposition.contains_key( id ) {
                proposition.insert( *id, HashSet::new() );
            }
            proposition.get_mut(id).unwrap().insert(i.clone());
        }
    }
    
    proposition
}

fn filter_transition_destination( t: &TransitionSys, states_allowed: &HashSet<Vec<i32>> ) -> TransitionSys {

    let mut tsys_filtered = TransitionSys::default();
    
    for (k,v) in t.g.e.iter() {
        if !tsys_filtered.g.e.contains_key( k ) {
            tsys_filtered.g.e.insert( k.clone(), HashSet::new() );
        }
        // println!("{:?}",i);
        for j in v.iter() {
            if states_allowed.contains(j) {
                tsys_filtered.g.e.get_mut( k ).unwrap().insert( j.clone() );
            }
        }
    }
    tsys_filtered
}

fn filter_transition_response( t: &TransitionSys, response_allowed: &HashMap<Vec<i32>, HashSet<Vec<i32>>> ) -> TransitionSys {

    let mut tsys_filtered = TransitionSys::default();
    
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
    tsys_filtered
}

pub fn value_func_controllable( tsys: TransitionSys, dest: HashSet<Vec<i32>>, current: Vec<i32> ){
    
    unimplemented!();
}

pub fn value_func_forced( tsys: TransitionSys, dest: HashSet<Vec<i32>>, current: Vec<i32> ){
    unimplemented!();
}

pub fn ControllablePredecessor() -> HashSet<Vec<i32>> {
    unimplemented!();
    let mut ret = vec![];
    for i in s.iter() {
        if value_func( tsys, dest, i ) < std::i32::MAX {
            ret.push(i);
        }
    }
    ret
}

fn compute_winning_set( tsys_ss_resp: &TransitionSys, task_states: &HashMap<usize,HashSet<Vec<i32>>> ) -> ( Vec< Vec<i32> >, HashMap<usize,HashSet<Vec<i32>>> ) {

    let n = task_states.len();
    if n == 0 {
        return ( vec![], HashMap::new() )
    }
    
    while true {
        for (k,v) in task_states.iter().take(n-1) {
            //todo
            
        }
    }
    unimplemented!();
}

fn main() {
    println!( "ltl_fragment_planning, version {}", ts::version() );

    let proposition_next_state_response = {};
        
    //create initial transition system
    let d = 10i32;
    
    let (s_map_1, g_map_1) = Graph::build_2d_map( d );
    let (s_map_2, g_map_2) = Graph::build_2d_map( d );

    //state space is agent_position x obstacle_position:
    //[ pos_agent_x, pos_agent_y, pos_obstacle_x, pos_obstacle_y ]
    let (mut s, mut g) = product_space( (s_map_1,g_map_1), s_map_2 );

    //agent position initialize to (0,0)
    s.s[0] = 0;
    s.s[1] = 0;
    //obstacle position initialize to (d/2,d/2)
    s.s[2] = d/2;
    s.s[3] = d/2;

    let tsys_orig = TransitionSys { s: s, g: g };
    
    // map (9,0) represents south west corner
    // A:= agent, O:=moving obstruction,
    // X:=stationary obstruction, T:=task
    // O allowed to move within rectangle (4,1), (8,7)
    //
    //   0123456789
    // 0 A              
    // 1    X   X
    // 2
    // 3   X   T          
    // 4   X
    // 5      O  
    // 6      X  X   
    // 7       
    // 8 T X     
    // 9          T

    // safety propositions ---------------------
    // do not collide with O and X
    
    // propositions for next step response:
    // for agent: (0,0) -> (0,1)
    //            (0,1) -> (0,2)
    //            (0,2) -> (0,3)

    // steady state next step response:
    // for agent pos: (9,8) -> (8,8)
    //                (8,8) -> (8,9)

    // persistence proposition:
    // remain in rectangle (3,0) bounded by (9,9)
    
    // task proposition:
    // visit each of T's infinitely often

    //------------------------------------------

    // println!("{:?}",tsys_orig.g);

    //add propositions for next step response-----
    let mut next_step_response : HashMap<(i32,i32), HashSet<(i32,i32)> > = HashMap::new();
    next_step_response.insert( (0,0), [(0,1)].iter().cloned().collect() );
    next_step_response.insert( (0,1), [(0,2)].iter().cloned().collect() );
    next_step_response.insert( (0,2), [(0,3)].iter().cloned().collect() );
    
    let next_step_response_allowed = generate_next_step_response_constraints( &tsys_orig.s, &next_step_response );
    // println!("{:?}", next_step_response_allowed );

    let tsys_resp = filter_transition_response( &tsys_orig, &next_step_response_allowed );
    tsys_resp.query_transitions_possible( &vec![ 0, 0, 6, 6] );
    tsys_resp.query_transitions_possible( &vec![ 0, 1, 6, 6] );
    tsys_resp.query_transitions_possible( &vec![ 0, 2, 6, 6] );
    //-----------------------------------------
    
    //add propositions for safety--------------
    let mut stationary_obstacles = HashSet::new();
    stationary_obstacles.insert( vec![1,3] );
    stationary_obstacles.insert( vec![1,7] );
    stationary_obstacles.insert( vec![3,2] );
    stationary_obstacles.insert( vec![4,2] );
    stationary_obstacles.insert( vec![6,5] );
    stationary_obstacles.insert( vec![6,8] );
    stationary_obstacles.insert( vec![8,2] );

    let moving_obstacle_range = ((4,1),(8,7));
    let states_satisfy_safety = generate_safety_constraints(&tsys_orig.s, &stationary_obstacles, moving_obstacle_range);
    // println!("{:?}",states_satisfy_safety);
    
    let tsys_safety = filter_transition_destination( &tsys_resp, &states_satisfy_safety );
    tsys_safety.query_transitions_possible( &vec![ 1, 2, 6, 6] );
    tsys_safety.query_transitions_possible( &vec![ 2, 3, 6, 6] );
    //-------------------------------------------

    //add propositions for persistence----------------
    let persistence_range = ((3,0),(9,9));
    let states_satisfy_persistence = generate_persistence_constraints(&tsys_orig.s, persistence_range);
    
    let tsys_pers = filter_transition_destination( &tsys_safety, &states_satisfy_persistence );
    tsys_pers.query_transitions_possible( &vec![ 3, 3, 6, 6] );
    //------------------------------------------------
    
    //add propositions for steady state response------
    let mut ss_next_step_response : HashMap<(i32,i32), HashSet<(i32,i32)> > = HashMap::new();
    ss_next_step_response.insert( (9,8), [(8,8)].iter().cloned().collect() );
    ss_next_step_response.insert( (8,8), [(8,9)].iter().cloned().collect() );

    let ss_next_step_response_allowed = generate_next_step_response_constraints( &tsys_orig.s, &ss_next_step_response );

    let tsys_ss_resp = filter_transition_response( &tsys_pers, &ss_next_step_response_allowed );
    tsys_ss_resp.query_transitions_possible( &vec![ 9, 8, 6, 6] );
    tsys_ss_resp.query_transitions_possible( &vec![ 8, 8, 6, 6] );
    //------------------------------------------------

    //collect all states for task---------------------
    let mut task_pos = HashSet::new();
    task_pos.insert( vec![3,6] );
    task_pos.insert( vec![8,0] );
    task_pos.insert( vec![9,9] );
    let task_states = generate_task_states(&tsys_orig.s, &task_pos );
    println!("task_groups len: {}", task_states.len());
    //------------------------------------------------

    let (w, task_groups) = compute_winning_set( &tsys_ss_resp, &task_states );
    
}
