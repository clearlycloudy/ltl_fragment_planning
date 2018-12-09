use std::collections::{HashMap,HashSet,BinaryHeap};

use crate::graph::Graph;
use crate::states::States;

pub fn version()-> & 'static str {
    "0.0.0"
}

#[derive(Debug,Default,Clone)]
pub struct TransitionSys {
    pub s: States,
    pub g: Graph,
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
    pub fn query_transitions_possible_partial( &self, state: &Vec<i32> ) {
        for (k,v) in self.g.e.iter() {
            let a = k.iter().take(state.len());
            let mut equal = true;
            for (item1,item2) in a.zip(state.iter()) {
                if *item1 != *item2 {
                    equal = false;
                }
            }
            if equal {
                println!("transition for: {:?}", k );
                for i in v {
                    println!("{:?}", i );
                }
            }
        }
    }
    pub fn query_states_partial_nonempty( &self, state: &Vec<i32> ) -> HashSet<Vec<i32>> {
        let mut ret = HashSet::new();
        for (k,v) in self.g.e.iter() {
            let a = k.iter().take(state.len());
            let mut equal = true;
            for (item1,item2) in a.zip(state.iter()) {
                if *item1 != *item2 {
                    equal = false;
                }
            }
            if equal {
                if v.len() != 0 {
                    ret.insert(k.clone());    
                }
            }
        }
        println!("states partial nonempty: {:?}", ret );
        ret
    }
}

