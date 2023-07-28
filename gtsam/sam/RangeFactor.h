/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  RangeFactor.h
 *  @brief Serializable factor induced by a range measurement
 *  @date July 2015
 *  @author Frank Dellaert
 *  @author Fan Jiang
 **/

#pragma once

#include <gtsam/nonlinear/ExpressionFactor.h>

namespace gtsam {

// forward declaration of Range functor, assumed partially specified
template <typename A1, typename A2>
struct Range;

/**
 * Binary factor for a range measurement
 * Works for any two types A1,A2 for which the functor Range<A1,A2>() is defined
 * @ingroup sam
 */
template <typename A1, typename A2 = A1, typename T = double>
class RangeFactor : public ExpressionFactorN<T, A1, A2> {
 private:
  typedef RangeFactor<A1, A2> This;
  typedef ExpressionFactorN<T, A1, A2> Base;

 public:
  /// default constructor
  RangeFactor() {}

  RangeFactor(Key key1, Key key2, T measured, const SharedNoiseModel& model)
      : Base({key1, key2}, model, measured) {
    this->initialize(expression({key1, key2}));
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  // Return measurement expression
  Expression<T> expression(const typename Base::ArrayNKeys& keys) const override {
    Expression<A1> a1_(keys[0]);
    Expression<A2> a2_(keys[1]);
    return Expression<T>(Range<A1, A2>(), a1_, a2_);
  }

  Vector evaluateError(const A1& a1, const A2& a2, 
      OptionalMatrixType H1 = OptionalNone,
      OptionalMatrixType H2 = OptionalNone) const {
    std::vector<Matrix> Hs(2);
    const auto& keys = Factor::keys();
    const Vector error = Base::unwhitenedError(
        {{keys[0], genericValue(a1)}, {keys[1], genericValue(a2)}},
        Hs);
    if (H1) *H1 = Hs[0];
    if (H2) *H2 = Hs[1];
    return error;
  }

  /// print
  void print(const std::string& s = "",
             const KeyFormatter& kf = DefaultKeyFormatter) const override {
    std::cout << s << "RangeFactor" << std::endl;
    Base::print(s, kf);
  }

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Base", boost::serialization::base_object<Base>(*this));
  }
#endif
};  // \ RangeFactor

/// traits
template <typename A1, typename A2, typename T>
struct traits<RangeFactor<A1, A2, T> >
    : public Testable<RangeFactor<A1, A2, T> > {};

/**
 * Binary factor for a range measurement, with a transform applied
 * @ingroup sam
 */
template <typename A1, typename A2 = A1,
          typename T = typename Range<A1, A2>::result_type>
class RangeFactorWithTransform : public ExpressionFactorN<T, A1, A2> {
 private:
  typedef RangeFactorWithTransform<A1, A2> This;
  typedef ExpressionFactorN<T, A1, A2> Base;

  A1 body_T_sensor_;  ///< The pose of the sensor in the body frame

 public:
  //// Default constructor
  RangeFactorWithTransform() {}

  RangeFactorWithTransform(Key key1, Key key2, T measured,
                           const SharedNoiseModel& model,
                           const A1& body_T_sensor)
      : Base({key1, key2}, model, measured), body_T_sensor_(body_T_sensor) {
    this->initialize(expression({key1, key2}));
  }

  ~RangeFactorWithTransform() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  // Return measurement expression
  Expression<T> expression(const typename Base::ArrayNKeys& keys) const override {
    Expression<A1> body_T_sensor__(body_T_sensor_);
    Expression<A1> nav_T_body_(keys[0]);
    Expression<A1> nav_T_sensor_(traits<A1>::Compose, nav_T_body_,
                                 body_T_sensor__);
    Expression<A2> a2_(keys[1]);
    return Expression<T>(Range<A1, A2>(), nav_T_sensor_, a2_);
  }

  Vector evaluateError(const A1& a1, const A2& a2,
      OptionalMatrixType H1 = OptionalNone, OptionalMatrixType H2 = OptionalNone) const {
    std::vector<Matrix> Hs(2);
    const auto &keys = Factor::keys();
    const Vector error = Base::unwhitenedError(
      {{keys[0], genericValue(a1)}, {keys[1], genericValue(a2)}}, 
      Hs);
    if (H1) *H1 = Hs[0];
    if (H2) *H2 = Hs[1];
    return error;
  }

  // An evaluateError overload to accept matrices (Matrix&) and pass it to the
  // OptionalMatrixType evaluateError overload
  Vector evaluateError(const A1& a1, const A2& a2, Matrix& H1, Matrix& H2) const {
	return evaluateError(a1, a2, &H1, &H2);
  }

  /** print contents */
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "RangeFactorWithTransform" << std::endl;
    this->body_T_sensor_.print("  sensor pose in body frame: ");
    Base::print(s, keyFormatter);
  }

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  /** Serialization function */
  template <typename ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // **IMPORTANT** We need to (de)serialize parameters before the base class,
    // since it calls expression() and we need all parameters ready at that
    // point.
    ar& BOOST_SERIALIZATION_NVP(body_T_sensor_);
    ar& boost::serialization::make_nvp(
        "Base", boost::serialization::base_object<Base>(*this));
  }
#endif
};  // \ RangeFactorWithTransform

/// traits
template <typename A1, typename A2, typename T>
struct traits<RangeFactorWithTransform<A1, A2, T> >
    : public Testable<RangeFactorWithTransform<A1, A2, T> > {};

/**
 * Binary factor for a range measurement, with a transform applied, and a bias
 * @ingroup sam
 */
template <typename A1, typename A2 = A1,
          typename T = typename Range<A1, A2>::result_type>
class RangeFactorWithTransformBias : public ExpressionFactorN<T, A1, A2, T> {
 private:
  typedef RangeFactorWithTransformBias<A1, A2> This;
  typedef ExpressionFactorN<T, A1, A2, T> Base;

  A1 body_T_sensor_;  ///< The pose of the sensor in the body frame

 public:
  //// Default constructor
  RangeFactorWithTransformBias() {}

  RangeFactorWithTransformBias(Key key1, Key key2, Key bias, T measured,
                           const SharedNoiseModel& model,
                           const A1& body_T_sensor)
      : Base({key1, key2, bias}, model, measured), body_T_sensor_(body_T_sensor) {
    this->initialize(expression({key1, key2, bias}));
  }

  ~RangeFactorWithTransformBias() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  // Return measurement expression
  Expression<T> expression(const typename Base::ArrayNKeys& keys) const override {
    Expression<A1> body_T_sensor__(body_T_sensor_);
    Expression<A1> nav_T_body_(keys[0]);
    Expression<A1> nav_T_sensor_(traits<A1>::Compose, nav_T_body_,
                                 body_T_sensor__);
    Expression<A2> a2_(keys[1]);
    Expression<T> bias_(keys[2]);
    return Expression<T>(Range<A1, A2>(), nav_T_sensor_, a2_) - bias_;
  }

  Vector evaluateError(const A1& a1, const A2& a2, const T& bias,
      OptionalMatrixType H1 = OptionalNone, OptionalMatrixType H2 = OptionalNone,
      OptionalMatrixType H3 = OptionalNone) const {
    std::vector<Matrix> Hs(3);
    const auto &keys = Factor::keys();
    const Vector error = Base::unwhitenedError(
      {{keys[0], genericValue(a1)}, {keys[1], genericValue(a2)}, {keys[2], genericValue(bias)}}, 
      Hs);
    if (H1) *H1 = Hs[0];
    if (H2) *H2 = Hs[1];
    if (H3) *H3 = Hs[2];
    return error;
  }

  // An evaluateError overload to accept matrices (Matrix&) and pass it to the
  // OptionalMatrixType evaluateError overload
  Vector evaluateError(const A1& a1, const A2& a2, const T& bias, Matrix& H1, Matrix& H2, Matrix& H3) const {
	return evaluateError(a1, a2, bias, &H1, &H2, &H3);
  }

  /** print contents */
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "RangeFactorWithTransformBias" << std::endl;
    this->body_T_sensor_.print("  sensor pose in body frame: ");
    Base::print(s, keyFormatter);
  }

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  /** Serialization function */
  template <typename ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // **IMPORTANT** We need to (de)serialize parameters before the base class,
    // since it calls expression() and we need all parameters ready at that
    // point.
    ar& BOOST_SERIALIZATION_NVP(body_T_sensor_);
    ar& boost::serialization::make_nvp(
        "Base", boost::serialization::base_object<Base>(*this));
  }
#endif
};  // \ RangeFactorWithTransformBias

/// traits
template <typename A1, typename A2, typename T>
struct traits<RangeFactorWithTransformBias<A1, A2, T> >
    : public Testable<RangeFactorWithTransformBias<A1, A2, T> > {};


/**
 * Binary factor for a range measurement, with a transform applied, and a bias
 * @ingroup sam
 */
template <typename A1, typename A2 = A1,
          typename T = typename Range<A1, A2>::result_type>
class RangeFactorWithTransformBiasAsymmetric : public ExpressionFactorN<T, A1, A2, T> {
 private:
  typedef RangeFactorWithTransformBiasAsymmetric<A1, A2> This;
  typedef ExpressionFactorN<T, A1, A2, T> Base;

  A1 body_T_sensor_;  ///< The pose of the sensor in the body frame
  T max_allowed_negativity_;

 public:
  //// Default constructor
  RangeFactorWithTransformBiasAsymmetric() {}

  RangeFactorWithTransformBiasAsymmetric(Key key1, Key key2, Key bias, T measured,
                           const SharedNoiseModel& model,
                           const A1& body_T_sensor,
                           T max_allowed_negativity)
      : Base({key1, key2, bias}, model, measured), body_T_sensor_(body_T_sensor), max_allowed_negativity_(max_allowed_negativity) {
    this->initialize(expression({key1, key2, bias}));
  }

  ~RangeFactorWithTransformBiasAsymmetric() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  // Return measurement expression
  Expression<T> expression(const typename Base::ArrayNKeys& keys) const override {
    Expression<A1> body_T_sensor__(body_T_sensor_);
    Expression<A1> nav_T_body_(keys[0]);
    Expression<A1> nav_T_sensor_(traits<A1>::Compose, nav_T_body_,
                                 body_T_sensor__);
    Expression<A2> a2_(keys[1]);
    Expression<T> bias_(keys[2]);
    return Expression<T>(Range<A1, A2>(), nav_T_sensor_, a2_) - bias_;
  }

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) -> Local(h(x),z) \f$.
   * We override this method to provide
   * both the function evaluation and its derivative(s) in H.
   */
  Vector unwhitenedError(const Values& x, OptionalMatrixVecType H = nullptr) const override {
    if (H) {
      const Vector value = Base::unwhitenedError(x, H);

      if (value[0] < max_allowed_negativity_) {
        (*H)[0].setZero();
        (*H)[1].setZero();
        (*H)[2].setZero();

        return (Vector(1) << 0.0).finished();
      }

      return value;
    } else {
      const Vector value = Base::unwhitenedError(x, H);
      if (value[0] < max_allowed_negativity_) {
        return (Vector(1) << 0.0).finished();
      }
      return value;
    }
  }

  // An evaluateError overload to accept matrices (Matrix&) and pass it to the
  // OptionalMatrixType evaluateError overload
  Vector evaluateError(const A1& a1, const A2& a2, const T& bias, Matrix& H1, Matrix& H2, Matrix& H3) const {
	return evaluateError(a1, a2, bias, &H1, &H2, &H3);
  }

  /** print contents */
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "RangeFactorWithTransformBiasAsymmetric" << std::endl;
    this->body_T_sensor_.print("  sensor pose in body frame: ");
    Base::print(s, keyFormatter);
  }

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  /** Serialization function */
  template <typename ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // **IMPORTANT** We need to (de)serialize parameters before the base class,
    // since it calls expression() and we need all parameters ready at that
    // point.
    ar& BOOST_SERIALIZATION_NVP(body_T_sensor_);
    ar& boost::serialization::make_nvp(
        "Base", boost::serialization::base_object<Base>(*this));
  }
#endif
};  // \ RangeFactorWithTransformBiasAsymmetric

/// traits
template <typename A1, typename A2, typename T>
struct traits<RangeFactorWithTransformBiasAsymmetric<A1, A2, T> >
    : public Testable<RangeFactorWithTransformBiasAsymmetric<A1, A2, T> > {};

}  // \ namespace gtsam
