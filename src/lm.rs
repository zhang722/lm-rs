use nalgebra as na;
use na::{DMatrix, DVector};

pub trait LMProblem {
    fn residual(&self, p: &DVector<f64>) -> DVector<f64>;
    fn jacobian(&self, p: &DVector<f64>) -> DMatrix<f64>;
}

pub fn levenberg_marquardt<P>(
    problem: P,
    init_param: DVector<f64>,
    max_iter: usize,
    tol: f64,
    lambda: f64,
    gamma: f64,
) -> DVector<f64>
where
    P: LMProblem,
{
    let mut x = init_param;
    let mut lambda_current = lambda;
    let mut iteration = 0;

    while iteration < max_iter {
        let fx = problem.residual(&x);
        let j = problem.jacobian(&x);

        let jtj = j.transpose() * &j;
        let mut jtj_with_lambda = jtj.clone();
        for i in 0..jtj_with_lambda.nrows() {
            jtj_with_lambda[(i, i)] += lambda_current;
        }

        let delta_x = jtj_with_lambda.lu().solve(&(j.transpose() * -fx.clone()));

        match delta_x {
            Some(dx) => {
                let new_x = &x + &dx;

                let fx_new = problem.residual(&new_x);
                let fx_norm = fx.norm();
                let fx_new_norm = fx_new.norm();

                if (fx_norm - fx_new_norm).abs() < tol {
                    break;
                }

                if fx_new_norm < fx_norm {
                    lambda_current /= gamma;
                    x = new_x;
                } else {
                    lambda_current *= gamma;
                }
            }
            None => {
                lambda_current *= gamma;
            }
        }

        iteration += 1;
    }

    x
}


// #[test]
// fn quadratic_function_fit() {
//     // Given a set of data points, we want to find the best quadratic function: y = ax^2 + bx + c
//     let data = vec![
//         (1.0, 3.0),
//         (2.0, 7.0),
//         (3.0, 13.0),
//         (4.0, 21.0),
//         (5.0, 31.0),
//     ];

//     let f = |params: &DVector<f64>| -> DVector<f64> {
//         let a = params[0];
//         let b = params[1];
//         let c = params[2];

//         let data = data
//             .iter()
//             .map(|(x, y)| a * x * x + b * x + c - y)
//             .collect::<Vec<_>>();
//         DVector::<f64>::from_vec(data) 
//     };

//     let jacobian = |params: &DVector<f64>| -> DMatrix<f64> {
//         let data = data.iter()
//             .map(|(x, _)| {
//                 na::RowDVector::from_row_slice(&[
//                     x * x, // Derivative w.r.t a
//                     *x,     // Derivative w.r.t b
//                     1.0,    // Derivative w.r.t c
//                 ])
//             })
//             .collect::<Vec<_>>();
//         DMatrix::from_rows(&data)
//     };

//     let x0 = DVector::from_vec(vec![200.0, 2.0, 3.0]);
//     let max_iter = 100;
//     let tol = 1e-8;
//     let lambda = 1e-3;
//     let gamma = 10.0;

//     let result = levenberg_marquardt(f, x0, jacobian, max_iter, tol, lambda, gamma);
//     let expected = DVector::from_vec(vec![1.0, 1.0, 1.0]);
//     println!("{result}");

//     assert!((result - expected).norm() < tol);
// }
