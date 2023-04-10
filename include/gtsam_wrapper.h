#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;

namespace gtsam_wrapper
{
    /** A convenient base class for creating your own NoiseModelFactor with 7
    * variables.  To derive from this class, implement evaluateError(). */
    template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5, class VALUE6, class VALUE7>
    class NoiseModelFactor7 : public NoiseModelFactor
    {

    public:
        // typedefs for value types pulled from keys
        typedef VALUE1 X1;
        typedef VALUE2 X2;
        typedef VALUE3 X3;
        typedef VALUE4 X4;
        typedef VALUE5 X5;
        typedef VALUE6 X6;
        typedef VALUE7 X7;

    protected:
        typedef NoiseModelFactor Base;
        typedef NoiseModelFactor7<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6, VALUE7> This;

    public:
        /**
         * Default Constructor for I/O
         */
        NoiseModelFactor7() {}

        /**
         * Constructor
         * @param noiseModel shared pointer to noise model
         * @param j1 key of the first variable
         * @param j2 key of the second variable
         * @param j3 key of the third variable
         * @param j4 key of the fourth variable
         * @param j5 key of the fifth variable
         * @param j6 key of the sixth variable
         * @param j7 key of the seventh variable
         */
        NoiseModelFactor7(const SharedNoiseModel &noiseModel, Key j1, Key j2, Key j3, Key j4, Key j5, Key j6, Key j7) : Base(noiseModel, cref_list_of<7>(j1)(j2)(j3)(j4)(j5)(j6)(j7)) {}

        virtual ~NoiseModelFactor7() {}

        /** methods to retrieve keys */
        inline Key key1() const { return keys_[0]; }
        inline Key key2() const { return keys_[1]; }
        inline Key key3() const { return keys_[2]; }
        inline Key key4() const { return keys_[3]; }
        inline Key key5() const { return keys_[4]; }
        inline Key key6() const { return keys_[5]; }
        inline Key key7() const { return keys_[6]; }

        /** 
         * Calls the 7-key specific version of evaluateError, which is pure virtual
         * so must be implemented in the derived class. 
         */
        virtual Vector unwhitenedError(const Values &x, boost::optional<std::vector<Matrix> &> H = boost::none) const
        {
            if (this->active(x))
            {
                if (H)
                    return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), (*H)[0], (*H)[1], (*H)[2], (*H)[3], (*H)[4], (*H)[5], (*H)[6]);
                else
                    return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]));
            }
            else
            {
                return Vector::Zero(this->dim());
            }
        }

        /**
         *  Override this method to finish implementing a 7-way factor.
         *  If any of the optional Matrix reference arguments are specified, it should compute
         *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
         */
        virtual Vector
        evaluateError(const X1 &, const X2 &, const X3 &, const X4 &, const X5 &, const X6 &, const X7 &,
                      boost::optional<Matrix &> H1 = boost::none,
                      boost::optional<Matrix &> H2 = boost::none,
                      boost::optional<Matrix &> H3 = boost::none,
                      boost::optional<Matrix &> H4 = boost::none,
                      boost::optional<Matrix &> H5 = boost::none,
                      boost::optional<Matrix &> H6 = boost::none,
                      boost::optional<Matrix &> H7 = boost::none) const = 0;

    private:
        /** Serialization function */
        friend class boost::serialization::access;
        template <class ARCHIVE>
        void serialize(ARCHIVE &ar, const unsigned int /*version*/)
        {
            ar &boost::serialization::make_nvp("NoiseModelFactor",
                                               boost::serialization::base_object<Base>(*this));
        }
    }; // \class NoiseModelFactor7

    template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5, class VALUE6, class VALUE7, class VALUE8>
    class NoiseModelFactor8 : public NoiseModelFactor
    {

    public:
        // typedefs for value types pulled from keys
        typedef VALUE1 X1;
        typedef VALUE2 X2;
        typedef VALUE3 X3;
        typedef VALUE4 X4;
        typedef VALUE5 X5;
        typedef VALUE6 X6;
        typedef VALUE7 X7;
        typedef VALUE8 X8;

    protected:
        typedef NoiseModelFactor Base;
        typedef NoiseModelFactor8<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6, VALUE7, VALUE8> This;

    public:
        /**
         * Default Constructor for I/O
         */
        NoiseModelFactor8() {}

        /**
         * Constructor
         * @param noiseModel shared pointer to noise model
         * @param j1 key of the first variable
         * @param j2 key of the second variable
         * @param j3 key of the third variable
         * @param j4 key of the fourth variable
         * @param j5 key of the fifth variable
         * @param j6 key of the sixth variable
         * @param j7 key of the seventh variable
         * @param j8 key of the eigth variable
         */
        NoiseModelFactor8(const SharedNoiseModel &noiseModel, Key j1, Key j2, Key j3, Key j4, Key j5, Key j6, Key j7, Key j8) : Base(noiseModel, cref_list_of<7>(j1)(j2)(j3)(j4)(j5)(j6)(j7)(j8)) {}

        virtual ~NoiseModelFactor8() {}

        /** methods to retrieve keys */
        inline Key key1() const { return keys_[0]; }
        inline Key key2() const { return keys_[1]; }
        inline Key key3() const { return keys_[2]; }
        inline Key key4() const { return keys_[3]; }
        inline Key key5() const { return keys_[4]; }
        inline Key key6() const { return keys_[5]; }
        inline Key key7() const { return keys_[6]; }
        inline Key key8() const { return keys_[7]; }

        /** 
         * Calls the 8-key specific version of evaluateError, which is pure virtual
         * so must be implemented in the derived class. 
         */
        virtual Vector unwhitenedError(const Values &x, boost::optional<std::vector<Matrix> &> H = boost::none) const
        {
            if (this->active(x))
            {
                if (H)
                    return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]), (*H)[0], (*H)[1], (*H)[2], (*H)[3], (*H)[4], (*H)[5], (*H)[6], (*H)[7]);
                else
                    return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]));
            }
            else
            {
                return Vector::Zero(this->dim());
            }
        }

        /**
         *  Override this method to finish implementing a 8-way factor.
         *  If any of the optional Matrix reference arguments are specified, it should compute
         *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
         */
        virtual Vector
        evaluateError(const X1 &, const X2 &, const X3 &, const X4 &, const X5 &, const X6 &, const X7 &, const X8 &,
                      boost::optional<Matrix &> H1 = boost::none,
                      boost::optional<Matrix &> H2 = boost::none,
                      boost::optional<Matrix &> H3 = boost::none,
                      boost::optional<Matrix &> H4 = boost::none,
                      boost::optional<Matrix &> H5 = boost::none,
                      boost::optional<Matrix &> H6 = boost::none,
                      boost::optional<Matrix &> H7 = boost::none,
                      boost::optional<Matrix &> H8 = boost::none) const = 0;

    private:
        /** Serialization function */
        friend class boost::serialization::access;
        template <class ARCHIVE>
        void serialize(ARCHIVE &ar, const unsigned int /*version*/)
        {
            ar &boost::serialization::make_nvp("NoiseModelFactor",
                                               boost::serialization::base_object<Base>(*this));
        }
    }; // \class NoiseModelFactor8

    template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5, class VALUE6, class VALUE7, class VALUE8, class VALUE9>
    class NoiseModelFactor9 : public NoiseModelFactor
    {

    public:
        // typedefs for value types pulled from keys
        typedef VALUE1 X1;
        typedef VALUE2 X2;
        typedef VALUE3 X3;
        typedef VALUE4 X4;
        typedef VALUE5 X5;
        typedef VALUE6 X6;
        typedef VALUE7 X7;
        typedef VALUE8 X8;
        typedef VALUE7 X9;


    protected:
        typedef NoiseModelFactor Base;
        typedef NoiseModelFactor9<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6, VALUE7, VALUE8, VALUE9> This;

    public:
        /**
         * Default Constructor for I/O
         */
        NoiseModelFactor9() {}

        /**
         * Constructor
         * @param noiseModel shared pointer to noise model
         * @param j1 key of the first variable
         * @param j2 key of the second variable
         * @param j3 key of the third variable
         * @param j4 key of the fourth variable
         * @param j5 key of the fifth variable
         * @param j6 key of the sixth variable
         * @param j7 key of the seventh variable
         * @param j8 key of the eigth variable
         * @param j9 key of the seventh variable
         */
        NoiseModelFactor9(const SharedNoiseModel &noiseModel, Key j1, Key j2, Key j3, Key j4, Key j5, Key j6, Key j7, Key j8, Key j9) 
            : Base(noiseModel, cref_list_of<7>(j1)(j2)(j3)(j4)(j5)(j6)(j7)(j8)(j9)) {}

        virtual ~NoiseModelFactor9() {}

        /** methods to retrieve keys */
        inline Key key1() const { return keys_[0]; }
        inline Key key2() const { return keys_[1]; }
        inline Key key3() const { return keys_[2]; }
        inline Key key4() const { return keys_[3]; }
        inline Key key5() const { return keys_[4]; }
        inline Key key6() const { return keys_[5]; }
        inline Key key7() const { return keys_[6]; }
        inline Key key8() const { return keys_[7]; }
        inline Key key9() const { return keys_[8]; }

        /** 
         * Calls the 10-key specific version of evaluateError, which is pure virtual
         * so must be implemented in the derived class. 
         */
        virtual Vector unwhitenedError(const Values &x, boost::optional<std::vector<Matrix> &> H = boost::none) const
        {
            if (this->active(x))
            {
                if (H)
                    return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), 
                        x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]), x.at<X9>(keys_[8]), 
                        (*H)[0], (*H)[1], (*H)[2], (*H)[3], (*H)[4], (*H)[5], (*H)[6], (*H)[7], (*H)[8]);
                else
                    return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), 
                    x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]), x.at<X9>(keys_[8]));
            }
            else
            {
                return Vector::Zero(this->dim());
            }
        }

        /**
         *  Override this method to finish implementing a 8-way factor.
         *  If any of the optional Matrix reference arguments are specified, it should compute
         *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
         */
        virtual Vector
        evaluateError(const X1 &, const X2 &, const X3 &, const X4 &, const X5 &, const X6 &, const X7 &, const X8 &,  const X9 &,
                      boost::optional<Matrix &> H1 = boost::none,
                      boost::optional<Matrix &> H2 = boost::none,
                      boost::optional<Matrix &> H3 = boost::none,
                      boost::optional<Matrix &> H4 = boost::none,
                      boost::optional<Matrix &> H5 = boost::none,
                      boost::optional<Matrix &> H6 = boost::none,
                      boost::optional<Matrix &> H7 = boost::none,
                      boost::optional<Matrix &> H8 = boost::none,
                      boost::optional<Matrix &> H9 = boost::none) const = 0;

    private:
        /** Serialization function */
        friend class boost::serialization::access;
        template <class ARCHIVE>
        void serialize(ARCHIVE &ar, const unsigned int /*version*/)
        {
            ar &boost::serialization::make_nvp("NoiseModelFactor",
                                               boost::serialization::base_object<Base>(*this));
        }
    }; // \class NoiseModelFactor9

    template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5, class VALUE6, class VALUE7, class VALUE8, class VALUE9, class VALUE10>
    class NoiseModelFactor10 : public NoiseModelFactor
    {

    public:
        // typedefs for value types pulled from keys
        typedef VALUE1 X1;
        typedef VALUE2 X2;
        typedef VALUE3 X3;
        typedef VALUE4 X4;
        typedef VALUE5 X5;
        typedef VALUE6 X6;
        typedef VALUE7 X7;
        typedef VALUE8 X8;
        typedef VALUE7 X9;
        typedef VALUE8 X10;

    protected:
        typedef NoiseModelFactor Base;
        typedef NoiseModelFactor10<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6, VALUE7, VALUE8, VALUE9, VALUE10> This;

    public:
        /**
         * Default Constructor for I/O
         */
        NoiseModelFactor10() {}

        /**
         * Constructor
         * @param noiseModel shared pointer to noise model
         * @param j1 key of the first variable
         * @param j2 key of the second variable
         * @param j3 key of the third variable
         * @param j4 key of the fourth variable
         * @param j5 key of the fifth variable
         * @param j6 key of the sixth variable
         * @param j7 key of the seventh variable
         * @param j8 key of the eigth variable
         * @param j9 key of the seventh variable
         * @param j10 key of the eigth variable
         */
        NoiseModelFactor10(const SharedNoiseModel &noiseModel, Key j1, Key j2, Key j3, Key j4, Key j5, Key j6, Key j7, Key j8, Key j9, Key j10) 
            : Base(noiseModel, cref_list_of<7>(j1)(j2)(j3)(j4)(j5)(j6)(j7)(j8)(j9)(j10)) {}

        virtual ~NoiseModelFactor10() {}

        /** methods to retrieve keys */
        inline Key key1() const { return keys_[0]; }
        inline Key key2() const { return keys_[1]; }
        inline Key key3() const { return keys_[2]; }
        inline Key key4() const { return keys_[3]; }
        inline Key key5() const { return keys_[4]; }
        inline Key key6() const { return keys_[5]; }
        inline Key key7() const { return keys_[6]; }
        inline Key key8() const { return keys_[7]; }
        inline Key key9() const { return keys_[8]; }
        inline Key key10() const { return keys_[9]; }

        /** 
         * Calls the 10-key specific version of evaluateError, which is pure virtual
         * so must be implemented in the derived class. 
         */
        virtual Vector unwhitenedError(const Values &x, boost::optional<std::vector<Matrix> &> H = boost::none) const
        {
            if (this->active(x))
            {
                if (H)
                    return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), 
                        x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]), x.at<X9>(keys_[8]), x.at<X10>(keys_[9]), 
                        (*H)[0], (*H)[1], (*H)[2], (*H)[3], (*H)[4], (*H)[5], (*H)[6], (*H)[7], (*H)[8], (*H)[9]);
                else
                    return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), 
                    x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]), x.at<X9>(keys_[8]), x.at<X10>(keys_[9]));
            }
            else
            {
                return Vector::Zero(this->dim());
            }
        }

        /**
         *  Override this method to finish implementing a 8-way factor.
         *  If any of the optional Matrix reference arguments are specified, it should compute
         *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
         */
        virtual Vector
        evaluateError(const X1 &, const X2 &, const X3 &, const X4 &, const X5 &, const X6 &, const X7 &, const X8 &,  const X9 &, const X10 &,
                      boost::optional<Matrix &> H1 = boost::none,
                      boost::optional<Matrix &> H2 = boost::none,
                      boost::optional<Matrix &> H3 = boost::none,
                      boost::optional<Matrix &> H4 = boost::none,
                      boost::optional<Matrix &> H5 = boost::none,
                      boost::optional<Matrix &> H6 = boost::none,
                      boost::optional<Matrix &> H7 = boost::none,
                      boost::optional<Matrix &> H8 = boost::none,
                      boost::optional<Matrix &> H9 = boost::none,
                      boost::optional<Matrix &> H10 = boost::none) const = 0;

    private:
        /** Serialization function */
        friend class boost::serialization::access;
        template <class ARCHIVE>
        void serialize(ARCHIVE &ar, const unsigned int /*version*/)
        {
            ar &boost::serialization::make_nvp("NoiseModelFactor",
                                               boost::serialization::base_object<Base>(*this));
        }
    }; // \class NoiseModelFactor10
};