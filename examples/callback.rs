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
    let mut lastnode = 0;

    move |ctx: Context| {
      let vars: Vec<_> = ctx.get_vars().cloned().collect();
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

        callback::Simplex => {
          let itcnt = try!(ctx.get_what(callback::Spx_ItrCnt)) as i64;
          if itcnt - lastiter >= 100 {
            lastiter = itcnt;

            let obj = try!(ctx.get_what(callback::Spx_ObjVal));
            let pinf = try!(ctx.get_what(callback::Spx_PrimInf));
            let dinf = try!(ctx.get_what(callback::Spx_DualInf));

            let ispert = try!(ctx.get_what(callback::Spx_IsPert));
            let ch = match ispert {
              0 => ' ',
              1 => 'S',
              _ => 'P'
            };

            println!("{} {}{} {} {}", itcnt, obj, ch, pinf, dinf);
          }
        }

        callback::MIP => {
          let nodecnt = try!(ctx.get_what(callback::MIP_NodCnt)) as i64;
          let objbst = try!(ctx.get_what(callback::MIP_ObjBst));
          let objbnd = try!(ctx.get_what(callback::MIP_ObjBnd));
          let solcnt = try!(ctx.get_what(callback::MIP_SolCnt));

          if nodecnt - lastnode >= 100 {
            lastnode = nodecnt;

            let actnodes = try!(ctx.get_what(callback::MIP_NodLeft));
            let itcnt = try!(ctx.get_what(callback::MIP_ItrCnt));
            let cutcnt = try!(ctx.get_what(callback::MIP_CutCnt));
            println!("{} {} {} {} {} {} {}",
                     nodecnt,
                     actnodes,
                     itcnt,
                     objbst,
                     objbnd,
                     solcnt,
                     cutcnt);
          }

          if (objbst - objbnd).abs() < 0.1 * (1.0 + objbst.abs()) {
            println!("Stop early - 10% gap achived");
            ctx.terminate();
          }

          if nodecnt >= 10000 && solcnt != 0 {
            println!("Stop early - 10000 nodes explored");
            ctx.terminate();
          }
        }

        callback::MIPSol => {
          let nodecnt = try!(ctx.get_what(callback::MIPSol_NodCnt)) as i64;
          let obj = try!(ctx.get_what(callback::MIPSol_Obj));
          let solcnt = try!(ctx.get_what(callback::MIPSol_SolCnt)) as i64;

          let x = try!(ctx.get_solution(vars.as_slice()));
          println!("**** New solution at node {}, obj {}, sol {}, x[0] = {} ****",
                   nodecnt,
                   obj,
                   solcnt,
                   x[0]);
        }

        callback::MIPNode => {
          println!("**** New node ****");
          if Status::from(try!(ctx.get_what(callback::MIP_NodeStatus))) == Status::Optimal {
            let x = try!(ctx.get_node_rel(vars.as_slice()));
            try!(ctx.set_solution(x.as_slice()));
          }
        }

        callback::Barrier => (),

        callback::Message => {
          let msg = try!(ctx.get_msg_string());
          println!("{}", msg);
        }
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
