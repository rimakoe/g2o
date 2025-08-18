// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "types_icp.h"

#include <Eigen/Geometry>
#include <Eigen/LU>

#include "g2o/core/eigen_types.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/misc.h"

namespace g2o {

G2O_REGISTER_TYPE_GROUP(icp);
G2O_REGISTER_TYPE_NAME("EDGE_V_V_GICP", EdgeVVGicp);

const Matrix3 EdgeVVGicp::kDRidx =
    (Matrix3() << 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0)
        .finished();  // differential quat matrices
const Matrix3 EdgeVVGicp::kDRidy =
    (Matrix3() << 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0)
        .finished();  // differential quat matrices
const Matrix3 EdgeVVGicp::kDRidz =
    (Matrix3() << 0.0, 2.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        .finished();  // differential quat matrices
const Matrix3 VertexSCam::kDRidx =
    (Matrix3() << 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0)
        .finished();  // differential quat matrices
const Matrix3 VertexSCam::kDRidy =
    (Matrix3() << 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0)
        .finished();  // differential quat matrices
const Matrix3 VertexSCam::kDRidz =
    (Matrix3() << 0.0, 2.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        .finished();  // differential quat matrices

Matrix3 VertexSCam::kcam_;
double VertexSCam::baseline_;

//
// Rigid 3D constraint between poses, given fixed point offsets
//

// return the error estimate as a 3-vector
void EdgeVVGicp::computeError() {
  // from <ViewPoint> to <Point>
  const VertexSE3* vp0 = vertexXnRaw<0>();
  const VertexSE3* vp1 = vertexXnRaw<1>();

  // get vp1 point into vp0 frame
  // could be more efficient if we computed this transform just once
  Vector3 p1;
  p1 = vp1->estimate() * measurement().pos1;
  p1 = vp0->estimate().inverse() * p1;

  Vector3 n1;
  n1 = vp1->estimate().rotation().matrix() * measurement().normal1;
  n1 = vp0->estimate().inverse().rotation().matrix() * n1;

  Vector3 cache1(1.0, 0.0, 0.0);
  cache1 = vp1->estimate().rotation().matrix() * cache1;

  Vector3 t1 = cache1.cross(n1).normalized();
  Vector3 t2 = t1.cross(n1).normalized();

  Vector3 p2 = p1 + t1 * 1.0;
  Vector3 p3 = p1 + t2 * 1.0;
  // get their difference
  // this is simple Euclidean distance, for now
  error_(0) = measurement().normal0.dot(p1 - measurement().pos0);
  error_(1) = measurement().normal0.dot(p2 - measurement().pos0);
  error_(2) = measurement().normal0.dot(p3 - measurement().pos0);

  if (!pl_pl) return;

  // re-define the information matrix
  // topLeftCorner<3,3>() is the rotation()
  const Matrix3 transform = (vp0->estimate().inverse() * vp1->estimate())
                                .matrix()
                                .topLeftCorner<3, 3>();
  information() = (cov0 + transform * cov1 * transform.transpose()).inverse();
}

#ifdef GICP_ANALYTIC_JACOBIANS

void EdgeVVGicp::linearizeOplus() {
  VertexSE3* vp0 = vertexXnRaw<0>();
  VertexSE3* vp1 = vertexXnRaw<1>();

  Vector3 p1;
  p1 = vp1->estimate() * measurement().pos1;
  p1 = vp0->estimate().inverse() * p1;

  Vector3 n1;
  n1 = vp1->estimate().rotation().matrix() * measurement().normal1;
  n1 = vp0->estimate().inverse().rotation().matrix() * n1;

  Vector3 cache1(1.0, 0.0, 0.0);
  cache1 = vp1->estimate().rotation().matrix() * cache1;

  Vector3 t1 = cache1.cross(n1).normalized();
  Vector3 t2 = t1.cross(n1).normalized();

  Vector3 p2 = p1 + t1 * 1.0;
  Vector3 p3 = p1 + t2 * 1.0;

  Eigen::Vector3d n = measurement().normal0;
  n.normalize();

  Eigen::Vector3d pts[3] = {p1, p2, p3};

  if (!vp1->fixed()) {
    for (int i = 0; i < 3; ++i) {
      Eigen::Matrix3d skew;
      skew << 0, -pts[i].z(), pts[i].y(), pts[i].z(), 0, -pts[i].x(),
          -pts[i].y(), pts[i].x(), 0;

      jacobianOplusXj_.block<1, 3>(i, 3) = -n.transpose() * skew;
      jacobianOplusXj_.block<1, 3>(i, 0) = n.transpose();
    }
  }
}
#endif

//
// stereo camera functions
//

#ifdef SCAM_ANALYTIC_JACOBIANS
/**
 * \brief Jacobian for stereo projection
 */
void Edge_XYZ_VSC::linearizeOplus() {
  VertexSCam* vc = static_cast<VertexSCam*>(vertices_[1]);

  VertexPointXYZ* vp = static_cast<VertexPointXYZ*>(vertices_[0]);
  Vector4 pt, trans;
  pt.head<3>() = vp->estimate();
  pt(3) = 1.0;
  trans.head<3>() = vc->estimate().translation();
  trans(3) = 1.0;

  // first get the world point in camera coords
  Eigen::Matrix<double, 3, 1, Eigen::ColMajor> pc = vc->w2n * pt;

  // Jacobians wrt camera parameters
  // set d(quat-x) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  double px = pc(0);
  double py = pc(1);
  double pz = pc(2);
  double ipz2 = 1.0 / (pz * pz);
  assert(!std::isnan(ipz2) && "[SetJac] infinite jac");

  double ipz2fx = ipz2 * vc->Kcam(0, 0);  // Fx
  double ipz2fy = ipz2 * vc->Kcam(1, 1);  // Fy
  double b = vc->baseline;                // stereo baseline

  Eigen::Matrix<double, 3, 1, Eigen::ColMajor> pwt;

  // check for local vars
  pwt = (pt - trans)
            .head<3>();  // transform translations, use differential rotation

  // dx
  Eigen::Matrix<double, 3, 1, Eigen::ColMajor> dp =
      vc->dRdx * pwt;  // dR'/dq * [pw - t]
  jacobianOplusXj_(0, 3) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 3) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 3) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  // dy
  dp = vc->dRdy * pwt;  // dR'/dq * [pw - t]
  jacobianOplusXj_(0, 4) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 4) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 4) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  // dz
  dp = vc->dRdz * pwt;  // dR'/dq * [pw - t]
  jacobianOplusXj_(0, 5) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 5) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 5) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px

  // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  dp = -vc->w2n.col(0);  // dpc / dx
  jacobianOplusXj_(0, 0) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 0) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 0) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  dp = -vc->w2n.col(1);                          // dpc / dy
  jacobianOplusXj_(0, 1) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 1) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 1) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  dp = -vc->w2n.col(2);                          // dpc / dz
  jacobianOplusXj_(0, 2) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 2) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 2) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px

  // Jacobians wrt point parameters
  // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  dp = vc->w2n.col(0);  // dpc / dx
  jacobianOplusXi_(0, 0) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXi_(1, 0) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXi_(2, 0) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  dp = vc->w2n.col(1);                           // dpc / dy
  jacobianOplusXi_(0, 1) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXi_(1, 1) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXi_(2, 1) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  dp = vc->w2n.col(2);                           // dpc / dz
  jacobianOplusXi_(0, 2) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXi_(1, 2) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXi_(2, 2) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
}
#endif

void EdgeXyzVsc::computeError() {
  // from <Point> to <Cam>
  VertexPointXYZ* point = vertexXnRaw<0>();
  VertexSCam* cam = vertexXnRaw<1>();
  // cam->setAll();

  // calculate the projection
  Vector3 kp;
  cam->mapPoint(kp, point->estimate());

  // error, which is backwards from the normal observed - calculated
  // measurement_ is the measured projection
  error_ = kp - measurement_;
}

void VertexSCam::oplusImpl(const VectorX::MapType& update) {
  VertexSE3::oplusImpl(update);
  setAll();
}

void VertexSCam::transformW2F(Eigen::Matrix<double, 3, 4, Eigen::ColMajor>& m,
                              const Vector3& trans, const Quaternion& qrot) {
  m.block<3, 3>(0, 0) = qrot.toRotationMatrix().transpose();
  m.col(3).setZero();  // make sure there's no translation
  Vector4 tt;
  tt.head(3) = trans;
  tt[3] = 1.0;
  m.col(3) = -m * tt;
}

void VertexSCam::transformF2W(Eigen::Matrix<double, 3, 4, Eigen::ColMajor>& m,
                              const Vector3& trans, const Quaternion& qrot) {
  m.block<3, 3>(0, 0) = qrot.toRotationMatrix();
  m.col(3) = trans;
}

void VertexSCam::setKcam(double fx, double fy, double cx, double cy,
                         double tx) {
  kcam_.setZero();
  kcam_(0, 0) = fx;
  kcam_(1, 1) = fy;
  kcam_(0, 2) = cx;
  kcam_(1, 2) = cy;
  kcam_(2, 2) = 1.0;
  baseline_ = tx;
}

void VertexSCam::setTransform() {
  w2n = estimate().inverse().matrix().block<3, 4>(0, 0);
  // transformW2F(w2n,estimate().translation(), estimate().rotation());
}

void VertexSCam::setProjection() { w2i = kcam_ * w2n; }

void VertexSCam::setDr() {
  // inefficient, just for testing
  // use simple multiplications and additions for production code in
  // calculating dRdx,y,z for dS'*R', with dS the incremental change
  dRdx = kDRidx * w2n.block<3, 3>(0, 0);
  dRdy = kDRidy * w2n.block<3, 3>(0, 0);
  dRdz = kDRidz * w2n.block<3, 3>(0, 0);
}

void VertexSCam::setAll() {
  setTransform();
  setProjection();
  setDr();
}

void VertexSCam::mapPoint(Vector3& res, const Vector3& pt3) const {
  Vector4 pt;
  pt.head<3>() = pt3;
  pt(3) = cst(1.0);
  Vector3 p1 = w2i * pt;
  Vector3 p2 = w2n * pt;
  Vector3 pb(baseline_, 0, 0);

  double invp1 = cst(1.0) / p1(2);
  res.head<2>() = p1.head<2>() * invp1;

  // right camera px
  p2 = kcam_ * (p2 - pb);
  res(2) = p2(0) / p2(2);
}

}  // namespace g2o
