use std::collections::{HashMap,HashSet,BinaryHeap};
use std::cmp::{Ordering,Reverse};

use crate::states::States;

#[derive(Default,Debug,Clone)]
pub struct Graph {
    pub e: HashMap<Vec<i32>, HashSet<Vec<i32>> >,
    pub e_reverse: HashMap<Vec<i32>, HashSet<Vec<i32>> >,
}

#[derive(Clone,Copy,Debug,Eq,PartialEq,Hash)]
pub enum Action {
    N,
    S,
    W,
    E,
}

impl Graph {
    
    pub fn build_2d_map( d: i32, hs: &HashSet<Vec<i32>>, ranges_allowed: Vec<((i32,i32),(i32,i32))>, delete_self_transition: bool ) -> ( States, Graph ) {
        
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

        if delete_self_transition {
            g.prune_self_transition();
        }

        g.prune_destination_transition( hs );

        for i in ranges_allowed.iter() {
            g.retain_intersection_range( *i );
        }
        
        ( states, g )
    }

    fn prune_self_transition( & mut self ){
        for (k,v) in self.e.iter_mut() {
            v.remove( k );
        }
    }

    fn prune_destination_transition( & mut self, hs: &HashSet<Vec<i32>> ){
        for (k,v) in self.e.iter_mut() {
            let mut delete_list = vec![];
            for i in v.iter() {
                if hs.contains( i ) {
                    delete_list.push( i.clone() );
                }
            }
            for i in delete_list {
                v.remove( &i );
            }
        }
    }

    pub fn prune_destination_transition_for_one( & mut self, s: &Vec<i32>, hs: &HashSet<Vec<i32>> ){
        match self.e.get_mut( s ) {
            Some( x ) => {
                let mut delete_list = vec![];
                for i in x.iter() {
                    if hs.contains( i ) {
                        delete_list.push( i.clone() );
                    }
                }
                for i in delete_list {
                    x.remove( &i );
                }
            },
            _ => {},
        }
    }

    fn retain_intersection_range( & mut self, range: ((i32,i32),(i32,i32)) ){ //retain region that intersects inclusively
        for (k,v) in self.e.iter_mut() {
            let mut delete_list = vec![];
            for i in v.iter() {
                if i[0] < (range.0).0 || i[0] > (range.1).0 ||
                   i[1] < (range.0).1 || i[1] > (range.1).1 {
                        
                        delete_list.push( i.clone() );
                    }
            }
            for i in delete_list {
                v.remove( &i );
            }
        }
    }
    
    fn prune_intersection_range( & mut self, range: ((i32,i32),(i32,i32)) ){ //retain range that does not intersect
        for (k,v) in self.e.iter_mut() {
            let mut delete_list = vec![];
            for i in v.iter() {
                if i[0] >= (range.0).0 || i[0] <= (range.1).0 ||
                   i[1] >= (range.0).1 || i[1] <= (range.1).1 {
                    delete_list.push( i.clone() );
                }
            }
            for i in delete_list {
                v.remove( &i );
            }
        }
    }

    pub fn update_reverse_edge( & mut self ) {
        self.e_reverse.clear();
        for (k,v) in self.e.iter() {
            for i in v.iter() {
                self.e_reverse.entry(i.clone()).or_insert(HashSet::new()).insert( k.clone() );
            }
        }
    }

    pub fn get_state_parents( & self, s: &Vec<i32> ) -> HashSet<Vec<i32>> {
        match self.e_reverse.get( s ){
            Some(x) => {
                x.clone()
            },
            _ => {
                HashSet::new()
            },
        }
    }

    pub fn get_state_children( & self, s: &Vec<i32> ) -> HashSet<Vec<i32>> {
        match self.e.get( s ){
            Some(x) => {
                x.clone()
            },
            _ => {
                HashSet::new()
            },
        }
    }

    pub fn get_state_children_from_action( & self, s: &Vec<i32>, a: Action ) -> HashSet<Vec<i32>> {
        match self.e.get( s ){
            Some(x) => {
                let mut hs = HashSet::new();
                for i in x.iter(){
                    let valid = match a {
                        Action::N => {
                            if i[0]-s[0] == -1 {
                                true
                            } else {
                                false
                            }
                        },
                        Action::S => {
                            if i[0]-s[0] == 1 {
                                true
                            } else {
                                false
                            }
                        },
                        Action::W => {
                            if i[1]-s[1] == -1 {
                                true
                            } else {
                                false
                            }
                        },
                        _ => {
                            if i[1]-s[1] == 1 {
                                true
                            } else {
                                false
                            }
                        },
                    };
                    if valid {
                        hs.insert( i.clone() );
                    }
                }
                hs
            },
            _ => {
                HashSet::new()
            },
        }
    }
    
    //just append the state from s2 to the already existing codomain items in g1
    pub fn product_space( (s1,g1): (States,Graph), (s2,g2): (States,Graph) ) -> ( States, Graph ) {
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
                        //codomain2.push(j);
                        g_temp.e.get_mut( &k2 ).unwrap().insert( codomain2 );
                    }
                }
            }
            use std::mem;
            mem::swap( & mut g, & mut g_temp );
        }

        for (k,v) in g.e.iter_mut() {

            let mut v_new = HashSet::new();
            
            let k2 : Vec<_> = k.iter().skip( s1.s_dim.len() ).cloned().collect();
            let v_copy = v.clone();
            let v2 = match g2.e.get( &k2 ){
                Some(x) => {
                    x.clone()
                },
                _ => {
                    HashSet::new()
                },
            };
            for i in v_copy.iter() {
                for j in v2.iter() {
                    let mut a = i.clone();
                    a.extend_from_slice( j.as_slice() );
                    v_new.insert( a );
                }
            }
            *v = v_new;
        }

        // println!("{:?}", g );

        // for (k,v) in g.e.iter() {
        //     if (k[0],k[1],k[2],k[3]) == (1,1,4,5) {
        //         println!("{:?}", v.len());
        //         println!("k: {:?}-----",k);
        //         println!("{:?}", v);
        //     }
        // }
        
        ( s, g )
    }

}
