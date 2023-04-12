use std::{fs::File, io::Read};
use nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
#[derive(Debug)]
pub struct Problem {
    pub K: na::Matrix3<f64>,
    pub world_points: Vec<na::Point2<f64>>,
    pub extrinsics: Vec<na::Isometry3<f64>>,
    pub image_points: Vec<Vec<na::Point2<f64>>>,
}

// pub fn load(path: &str) -> Result<(Vec<na::Point3<f64>>, Vec<Vec<na::Point2<f64>>>), Box<dyn std::error::Error>> {
pub fn load(path: &str) -> Result<Problem, Box<dyn std::error::Error>> {
    // 打开文件
    let mut file = File::open(path).expect("Unable to open file");

    // 读取文件内容
    let mut content = String::new();
    file.read_to_string(&mut content).expect("Unable to read the file");

    let p: Problem = serde_json::from_str(&content)?;
    Ok(p)
}

#[test]
fn test_load() {
    load("problem.json").unwrap();
    assert!(1==2);
}