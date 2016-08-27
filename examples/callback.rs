// Copyright (c) 2016 Yusuke Sasaki
//
// This software is released under the MIT License.
// See http://opensource.org/licenses/mit-license.php or <LICENSE>.

extern crate gurobi;
use gurobi::*;


fn main() {
  let mut env = Env::new("callback.log").unwrap();
  env.set(param::OutputFlag, 0).unwrap();
  env.set(param::Heuristics, 0.0).unwrap();

  let mut model = env.read_model(&std::env::args().nth(1).unwrap()).unwrap();

  let callback = {
    let mut lastiter = 0;

    |ctx: Context| -> gurobi::Result<()> {
      let vars: Vec<_> = ctx.get_vars().collect();
      match ctx.get_where() {
        callback::Polling => {
          // Ignore polling callback
        }

        callback::PreSolve => {
          let cdels = try!(ctx.get_what(callback::Pre_ColDel));
          let rdels = try!(ctx.get_what(callback::Pre_RowDel));
          if cdels > 0 || rdels > 0 {
            println!("{} columns and {} rows are removed.", cdels, rdels);
          }
        }

        callback::Simplex => (),
        callback::MIP => (),
        callback::MIPSol => (),
        callback::MIPNode => (),
        callback::Barrier => (),
        callback::Message => (),
      }
      Ok(())
    }
  };
  model.optimize_with_callback(callback).unwrap();

  println!("\nOptimization complete");
  if model.get(attr::SolCount).unwrap() == 0 {
    println!("No solution found. optimization status = {:?}",
             model.status());
  } else {
    println!("Solution found. objective = {}",
             model.get(attr::ObjVal).unwrap());
    for v in model.get_vars() {
      let vname = v.get(&model, attr::VarName).unwrap();
      let value = v.get(&model, attr::X).unwrap();
      if value > 1e-25 {
        println!("  {}: {}", vname, value);
      }
    }
  }
}
