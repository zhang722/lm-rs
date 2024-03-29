use nalgebra as na;

use na::{DVector, DMatrix};
use crate::{lm, problem::Problem};

/// Produces a skew-symmetric or "cross-product matrix" from
/// a 3-vector. This is needed for the `exp_map` and `log_map`
/// functions
fn skew_sym(v: na::Vector3<f64>) -> na::Matrix3<f64> {
    let mut ss = na::Matrix3::zeros();
    ss[(0, 1)] = -v[2];
    ss[(0, 2)] = v[1];
    ss[(1, 0)] = v[2];
    ss[(1, 2)] = -v[0];
    ss[(2, 0)] = -v[1];
    ss[(2, 1)] = v[0];
    ss
}

/// Converts a 6-Vector Lie Algebra representation of a rigid body
/// transform to an NAlgebra Isometry (quaternion+translation pair)
///
/// This is largely taken from this paper:
/// https://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
fn exp_map(param_vector: &na::Vector6<f64>) -> na::Isometry3<f64> {
    let t = param_vector.fixed_view::<3, 1>(0, 0);
    let omega = param_vector.fixed_view::<3, 1>(3, 0);
    let theta = omega.norm();
    let half_theta = 0.5 * theta;
    let quat_axis = omega * half_theta.sin() / theta;
    let quat = if theta > 1e-6 {
        na::UnitQuaternion::from_quaternion(na::Quaternion::new(
            half_theta.cos(),
            quat_axis.x,
            quat_axis.y,
            quat_axis.z,
        ))
    } else {
        na::UnitQuaternion::identity()
    };

    let mut v = na::Matrix3::<f64>::identity();
    if theta > 1e-6 {
        let ssym_omega = skew_sym(omega.clone_owned());
        v += ssym_omega * (1.0 - theta.cos()) / (theta.powi(2))
            + (ssym_omega * ssym_omega) * ((theta - theta.sin()) / (theta.powi(3)));
    }

    let trans = na::Translation::from(v * t);

    na::Isometry3::from_parts(trans, quat)
}

/// Converts an NAlgebra Isometry to a 6-Vector Lie Algebra representation
/// of a rigid body transform.
///
/// This is largely taken from this paper:
/// https://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
fn log_map(input: &na::Isometry3<f64>) -> na::Vector6<f64> {
    let t: na::Vector3<f64> = input.translation.vector;

    let quat = input.rotation;
    let theta: f64 = 2.0 * (quat.scalar()).acos();
    let half_theta = 0.5 * theta;
    let mut omega = na::Vector3::<f64>::zeros();

    let mut v_inv = na::Matrix3::<f64>::identity();
    if theta > 1e-6 {
        omega = quat.vector() * theta / (half_theta.sin());
        let ssym_omega = skew_sym(omega);
        v_inv -= ssym_omega * 0.5;
        v_inv += ssym_omega * ssym_omega * (1.0 - half_theta * half_theta.cos() / half_theta.sin())
            / (theta * theta);
    }

    let mut ret = na::Vector6::<f64>::zeros();
    ret.fixed_view_mut::<3, 1>(0, 0).copy_from(&(v_inv * t));
    ret.fixed_view_mut::<3, 1>(3, 0).copy_from(&omega);

    ret
}

/// Produces the Jacobian of the exponential map of a lie algebra transform
/// that is then applied to a point with respect to the transform.
///
/// i.e.
/// d exp(t) * p
/// ------------
///      d t
///
/// The parameter 'transformed_point' is assumed to be transformed already
/// and thus: transformed_point = exp(t) * p
///
/// This is largely taken from this paper:
/// https://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
fn exp_map_jacobian(transformed_point: &na::Point3<f64>) -> na::Matrix3x6<f64> {
    let mut ss = na::Matrix3x6::zeros();
    ss.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&na::Matrix3::<f64>::identity());
    ss.fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&(-skew_sym(transformed_point.coords)));
    ss
}

/// Projects a point in camera coordinates into the image plane
/// producing a floating-point pixel value
fn project(
    params: &na::Vector4<f64>, /*fx, fy, cx, cy*/
    pt: &na::Point3<f64>,
) -> na::Point2<f64> {
    na::Point2::<f64>::new(
        params[0] * pt.x / pt.z + params[2], // fx * x / z + cx
        params[1] * pt.y / pt.z + params[3], // fy * y / z + cy
    )
}

/// Jacobian of the projection function with respect to the four camera
/// paramaters (fx, fy, cx, cy). The 'transformed_pt' is a point already in
/// or transformed to camera coordinates.
fn proj_jacobian_wrt_params(transformed_pt: &na::Point3<f64>) -> na::Matrix2x4<f64> {
    na::Matrix2x4::<f64>::new(
        transformed_pt.x / transformed_pt.z,
        0.0,
        1.0,
        0.0,
        0.0,
        transformed_pt.y / transformed_pt.z,
        0.0,
        1.0,
    )
}

/// Jacobian of the projection function with respect to the 3D point in camera
/// coordinates. The 'transformed_pt' is a point already in
/// or transformed to camera coordinates.
fn proj_jacobian_wrt_point(
    camera_model: &na::Vector4<f64>, /*fx, fy, cx, cy*/
    transformed_pt: &na::Point3<f64>,
) -> na::Matrix2x3<f64> {
    na::Matrix2x3::<f64>::new(
        camera_model[0] / transformed_pt.z,
        0.0,
        -transformed_pt.x * camera_model[0] / (transformed_pt.z.powi(2)),
        0.0,
        camera_model[1] / transformed_pt.z,
        -transformed_pt.y * camera_model[1] / (transformed_pt.z.powi(2)),
    )
}

/// Struct for holding data for calibration.
///
/// This struct implements the ArgminOp trait which allows it
/// to provide Jacobian and residual functions for the optimization.
struct Calibration<'a> {
    model_pts: &'a Vec<na::Point3<f64>>,
    image_pts_set: &'a Vec<Vec<na::Point2<f64>>>,
}

impl<'a> Calibration<'a> {
    /// Decode the camera model and transforms from the flattened parameter vector
    ///
    /// The convention in use is that the first four values are the camera parameters
    /// and each set of six values after is a transform (one per image). This convention
    /// is also followed in the Jacobian function below.
    fn decode_params(
        &self,
        param: &na::DVector<f64>,
    ) -> (na::Vector4<f64>, Vec<na::Isometry3<f64>>) {
        // Camera parameters are in the first four elements
        let camera_model: na::Vector4<f64> = param.fixed_view::<4, 1>(0, 0).clone_owned();

        // Following the camera parameters, for each image there
        // will be one 6D transform
        let transforms = self
            .image_pts_set
            .iter()
            .enumerate()
            .map(|(i, _)| {
                let lie_alg_transform: na::Vector6<f64> =
                    param.fixed_view::<6, 1>(4 + 6 * i, 0).clone_owned();
                // Convert to a useable na::Isometry
                exp_map(&lie_alg_transform)
            })
            .collect::<Vec<_>>();
        (camera_model, transforms)
    }
}

impl lm::LMProblem for Calibration<'_> {
    fn residual(&self, p: &na::DVector<f64>) -> na::DVector<f64> {
        // Get usable camera model and transforms from the parameter vector
        let (camera_model, transforms) = self.decode_params(p);

        let num_images = self.image_pts_set.len();
        let num_target_points = self.model_pts.len();
        let num_residuals = num_images * num_target_points;

        // Allocate big empty residual
        let mut residual = na::DVector::<f64>::zeros(num_residuals * 2);

        let mut residual_idx = 0;
        for (image_pts, transform) in self.image_pts_set.iter().zip(transforms.iter()) {
            for (observed_image_pt, target_pt) in image_pts.iter().zip(self.model_pts.iter()) {
                // Apply image formation model
                let transformed_point = transform * target_pt;
                let projected_pt = project(&camera_model, &transformed_point);

                // Populate residual vector two rows at time
                let individual_residual = projected_pt - observed_image_pt;
                residual
                    .fixed_view_mut::<2, 1>(residual_idx, 0)
                    .copy_from(&individual_residual);
                residual_idx += 2;
            }
        }

        residual
    }

    /// Calculate the Jacobian from the parameter vector
    ///
    /// param 'p' is the current estimate of the parameters-being-optimized
    /// at the current iteration
    fn jacobian(&self, p: &na::DVector<f64>) -> na::DMatrix<f64> {
        // Get usable camera model and transforms from the parameter vector
        let (camera_model, transforms) = self.decode_params(p);

        let num_images = self.image_pts_set.len();
        let num_target_points = self.model_pts.len();
        let num_residuals = num_images * num_target_points;
        let num_unknowns = 6 * num_images + 4;

        // Allocate big empty Jacobian
        let mut jacobian = na::DMatrix::<f64>::zeros(num_residuals * 2, num_unknowns);

        let mut residual_idx = 0;
        for (tform_idx, transform) in transforms.iter().enumerate() {
            for target_pt in self.model_pts.iter() {
                // Apply image formation model
                let transformed_point = transform * target_pt;

                // Populate Jacobian matrix two rows at time

                // Populate Jacobian part for the camera parameters
                jacobian
                    .fixed_view_mut::<2, 4>(
                        residual_idx,
                        0, /*first four columns are camera parameters*/
                    )
                    .copy_from(&proj_jacobian_wrt_params(&transformed_point));

                // Populate the Jacobian part for the transform
                let proj_jacobian_wrt_point =
                    proj_jacobian_wrt_point(&camera_model, &transformed_point);
                let transform_jacobian_wrt_transform = exp_map_jacobian(&transformed_point);

                // Transforms come after camera parameters in sets of six columns
                jacobian
                    .fixed_view_mut::<2, 6>(residual_idx, 4 + tform_idx * 6)
                    .copy_from(&(proj_jacobian_wrt_point * transform_jacobian_wrt_transform));

                residual_idx += 2;
            }
        }
        jacobian
    }

}

#[test]
fn test_opt() {
    let problem = crate::problem::load("problem.json").unwrap();

    // Ground truth camera model
    let camera_model = na::Vector4::<f64>::new(540.0, 540.0, 320.0, 240.0); // fx, fy, cx, cy

    // Ground truth camera-from-model transforms for three "images"
    let transforms = problem.extrinsics;
    let source_pts = problem.world_points.iter().map(|p| {
        na::Point3::<f64>::new(p.x, p.y, 0.0)
    }).collect::<Vec<_>>();
    let imaged_pts = problem.image_points.into_iter().take(3).collect::<Vec<_>>();

    // Create calibration parameters
    let cal_cost = Calibration {
        model_pts: &source_pts,
        image_pts_set: &imaged_pts,
    };

    // Populate a parameter vector with initial guess
    let mut init_param = na::DVector::<f64>::zeros(4 + imaged_pts.len() * 6);

    // Arbitrary guess for camera model
    init_param[0] = 550.0; // fx
    init_param[1] = 550.0; // fy
    init_param[2] = 320.0; // cx
    init_param[3] = 240.0; // cy

    // Arbitrary guess for poses (3m in front of the camera with no rotation)
    // We have to convert this to a 6D lie algebra element to populate the parameter
    // vector.
    let init_pose_lie = log_map(&na::Isometry3::translation(0.0, 0.0, 3.0));

    // Populate poses with guesses. This follows the same convention used in
    // `decode_params`
    // init_param
    //     .fixed_view_mut::<6, 1>(4, 0)
    //     .copy_from(&init_pose_lie);
    // init_param
    //     .fixed_view_mut::<6, 1>(4 + 6, 0)
    //     .copy_from(&init_pose_lie);
    // init_param
    //     .fixed_view_mut::<6, 1>(4 + 6 * 2, 0)
    //     .copy_from(&init_pose_lie);
    // init_param
    //     .fixed_view_mut::<6, 1>(4 + 6 * 3, 0)
    //     .copy_from(&init_pose_lie);
    // println!("{}", imaged_pts.len());
    for idx in 0..imaged_pts.len() {
        init_param
            .fixed_view_mut::<6, 1>(4 + idx * 6, 0)
            .copy_from(&init_pose_lie);
    }
    let max_iter = 300000;
    let tol = 1e-8;
    let lambda = 1e-3;
    let gamma = 10.0;

    let result = crate::lm::levenberg_marquardt(cal_cost, init_param, max_iter, tol, lambda, gamma);
    println!("{result}");

    // Print transforms
    for (i, t) in transforms.iter().enumerate() {
        eprintln!(
            "optimized result[{}]: {}\n",
            i,
            exp_map(
                &result
                    .fixed_view::<6, 1>(4 + 6 * i, 0)
                    .clone_owned()
            )
        );
    }
    assert!(1==2);
}