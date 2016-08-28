// Copyright (c) 2016 Yusuke Sasaki
//
// This software is released under the MIT License.
// See http://opensource.org/licenses/mit-license.php or <LICENSE>.

#![allow(dead_code)]

use ffi;
use itertools::Itertools;

use std::ops::Deref;
use std::mem::transmute;
use std::ptr::null;

use error::{Error, Result};
use model::{Model, Var, LinExpr, ConstrSense, ProxyBase};
use util;

// Location where the callback called.
const POLLING: i32 = 0;
const PRESOLVE: i32 = 1;
const SIMPLEX: i32 = 2;
const MIP: i32 = 3;
const MIPSOL: i32 = 4;
const MIPNODE: i32 = 5;
const MESSAGE: i32 = 6;
const BARRIER: i32 = 7;


const PRE_COLDEL: i32 = 1000;
const PRE_ROWDEL: i32 = 1001;
const PRE_SENCHG: i32 = 1002;
const PRE_BNDCHG: i32 = 1003;
const PRE_COECHG: i32 = 1004;

const SPX_ITRCNT: i32 = 2000;
const SPX_OBJVAL: i32 = 2001;
const SPX_PRIMINF: i32 = 2002;
const SPX_DUALINF: i32 = 2003;
const SPX_ISPERT: i32 = 2004;

const MIP_OBJBST: i32 = 3000;
const MIP_OBJBND: i32 = 3001;
const MIP_NODCNT: i32 = 3002;
const MIP_SOLCNT: i32 = 3003;
const MIP_CUTCNT: i32 = 3004;
const MIP_NODLFT: i32 = 3005;
const MIP_ITRCNT: i32 = 3006;
const MIP_OBJBNDC: i32 = 3007;

const MIPSOL_SOL: i32 = 4001;
const MIPSOL_OBJ: i32 = 4002;
const MIPSOL_OBJBST: i32 = 4003;
const MIPSOL_OBJBND: i32 = 4004;
const MIPSOL_NODCNT: i32 = 4005;
const MIPSOL_SOLCNT: i32 = 4006;
const MIPSOL_OBJBNDC: i32 = 4007;

const MIPNODE_STATUS: i32 = 5001;
const MIPNODE_REL: i32 = 5002;
const MIPNODE_OBJBST: i32 = 5003;
const MIPNODE_OBJBND: i32 = 5004;
const MIPNODE_NODCNT: i32 = 5005;
const MIPNODE_SOLCNT: i32 = 5006;
const MIPNODE_BRVAR: i32 = 5007;
const MIPNODE_OBJBNDC: i32 = 5008;

const MSG_STRING: i32 = 6001;
const RUNTIME: i32 = 6002;

const BARRIER_ITRCNT: i32 = 7001;
const BARRIER_PRIMOBJ: i32 = 7002;
const BARRIER_DUALOBJ: i32 = 7003;
const BARRIER_PRIMINF: i32 = 7004;
const BARRIER_DUALINF: i32 = 7005;
const BARRIER_COMPL: i32 = 7006;


#[derive(Debug, Clone)]
pub enum Where {
  Polling,
  PreSolve(i32),
  Simplex(i32),
  MIP(i32),
  MIPSol(i32),
  MIPNode(i32),
  Message(String),
  Barrier(i32)
}

impl Into<i32> for Where {
  fn into(self) -> i32 {
    match self {
      Where::Polling => POLLING,
      Where::PreSolve(_) => PRESOLVE,
      Where::Simplex(_) => SIMPLEX,
      Where::MIP(_) => MIP,
      Where::MIPSol(_) =>MIPSOL,
      Where::MIPNode(_) =>MIPNODE,
      Where::Message(_) =>MESSAGE,
      Where::Barrier(_) =>BARRIER,
    }
  }
}


/// a
pub struct Context<'a> {
  cbdata: *mut ffi::c_void,
  where_: Where,
  model: &'a Model<'a>
}


pub trait New<'a> {
  fn new(cbdata: *mut ffi::c_void, where_: i32, model: &'a Model<'a>) -> Context<'a>;
}

impl<'a> New<'a> for Context<'a> {
  fn new(cbdata: *mut ffi::c_void, where_: i32, model: &'a Model<'a>) -> Context<'a> {
    let where_ = match where_ {
      POLLING => Where::Polling,
      PRESOLVE => Where::PreSolve(0),
      SIMPLEX => Where::Simplex(0),
      MIP => Where::MIP(0),
      MIPSOL => Where::MIPSol(0),
      MIPNODE => Where::MIPNode(0),
      MESSAGE => Where::Message("a".to_owned()),
      BARRIER => Where::Barrier(0),
      _ => panic!("Invalid callback location. {}", where_),
    };

    Context {
      cbdata: cbdata,
      where_: where_,
      model: model
    }
  }
}


impl<'a> Context<'a> {
  /// a
  pub fn get_where(&self) -> Where { self.where_.clone() }

  /// a
  pub fn get_node_rel(&self, vars: &[Var]) -> Result<Vec<f64>> {
    // memo: only MIPNode && status == Optimal
    self.get_double_array(MIPNODE, MIPNODE_REL).map(|buf| vars.iter().map(|v| buf[v.index() as usize]).collect_vec())
  }

  /// a
  pub fn get_solution(&self, vars: &[Var]) -> Result<Vec<f64>> {
    self.get_double_array(MIPSOL, MIPSOL_SOL).map(|buf| vars.iter().map(|v| buf[v.index() as usize]).collect_vec())
  }

  /// Provide a new feasible solution for a MIP model.
  pub fn set_solution(&self, solution: &[f64]) -> Result<()> {
    if solution.len() < self.model.vars.len() {
      return Err(Error::InconsitentDims);
    }

    self.check_apicall(unsafe { ffi::GRBcbsolution(self.cbdata, solution.as_ptr()) })
  }

  /// Add a new cutting plane to the MIP model.
  pub fn add_cut(&self, lhs: LinExpr, sense: ConstrSense, rhs: f64) -> Result<()> {
    self.check_apicall(unsafe {
      ffi::GRBcbcut(self.cbdata,
                    lhs.coeff.len() as ffi::c_int,
                    lhs.vars.into_iter().map(|e| e.index()).collect_vec().as_ptr(),
                    lhs.coeff.as_ptr(),
                    sense.into(),
                    rhs - lhs.offset)
    })
  }

  /// Add a new lazy constraint to the MIP model.
  pub fn add_lazy(&self, lhs: LinExpr, sense: ConstrSense, rhs: f64) -> Result<()> {
    self.check_apicall(unsafe {
      ffi::GRBcblazy(self.cbdata,
                     lhs.coeff.len() as ffi::c_int,
                     lhs.vars.into_iter().map(|e| e.index()).collect_vec().as_ptr(),
                     lhs.coeff.as_ptr(),
                     sense.into(),
                     rhs - lhs.offset)
    })
  }


  fn get_int(&self, where_: i32, what: i32) -> Result<i32> {
    let mut buf = 0;
    self.check_apicall(unsafe { ffi::GRBcbget(self.cbdata, where_, what, transmute(&mut buf)) }).and(Ok(buf.into()))
  }

  fn get_double(&self, where_: i32, what: i32) -> Result<f64> {
    let mut buf = 0.0;
    self.check_apicall(unsafe { ffi::GRBcbget(self.cbdata, where_, what, transmute(&mut buf)) }).and(Ok(buf.into()))
  }

  fn get_double_array(&self, where_: i32, what: i32) -> Result<Vec<f64>> {
    let mut buf = vec![0.0; self.model.vars.len()];
    self.check_apicall(unsafe { ffi::GRBcbget(self.cbdata, where_, what, transmute(buf.as_mut_ptr())) }).and(Ok(buf))
  }

  fn get_string(&self, where_: i32, what: i32) -> Result<String> {
    let mut buf = null();
    self.check_apicall(unsafe { ffi::GRBcbget(self.cbdata, where_, what, transmute(&mut buf)) })
      .and(Ok(unsafe { util::from_c_str(buf) }))
  }

  fn check_apicall(&self, error: ffi::c_int) -> Result<()> {
    if error != 0 {
      return Err(Error::FromAPI("Callback error".to_owned(), 40000));
    }
    Ok(())
  }
}


impl<'a> Deref for Context<'a> {
  type Target = Model<'a>;
  fn deref(&self) -> &Model<'a> { self.model }
}
