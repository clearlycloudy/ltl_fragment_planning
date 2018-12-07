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

#[derive(Debug,Clone)]
pub struct States {
    s: Vec<i32>,
    s_dim: Vec<i32>, //indicates dimension of each state
}

pub struct TransitionSys {
    s: States,
    g: Graph,
}

fn main() {
    println!( "ltl_fragment_planning, version {}", ts::version() );

    let proposition_next_state_response = {};
        
    //create initial transition system
    let d = 2i32;
    
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

    // map (9,0) represents south west corner
    // A:= agent, O:=moving obstruction,
    // X:=stationary obstruction, T:=task
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

    // safety propositions:
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
    
    // println!("{:?}, {:?}", s, g );
    // for (k,v) in g.e.iter(){
    //     if k[0] == 1 && k[1] == 1 {
    //         println!("{:?}: {:?}", k, v );
    //     }
    // }

}
