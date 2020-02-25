
//yarp
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>

#include <WalkingControllers/StepAdaptationController/QPSolver.hpp>
#include <WalkingControllers/iDynTreeUtilities/Helper.h>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;
using namespace WalkingControllers;
QPSolver::QPSolver(const int& inputSize, const int& numberOfConstraints)
    :m_inputSize(inputSize), m_numberOfConstraints(numberOfConstraints)
{
    // instantiate the solver class
    m_QPSolver = std::make_unique<OsqpEigen::Solver>();

    //set the number of deceision variables of QP problem
    m_QPSolver->data()->setNumberOfVariables(inputSize);

    // set the number of all constraints includes inequality and equality constraints
    m_QPSolver->data()->setNumberOfConstraints(numberOfConstraints);

    m_QPSolver->settings()->setVerbosity(false);
    m_QPSolver->settings()->setPolish(true);

    m_constraintsMatrix.resize(numberOfConstraints, inputSize);
    m_upperBound.resize(numberOfConstraints);
    m_lowerBound.resize(numberOfConstraints);
    m_gradient.resize(inputSize);

    // set the constant elements of the constraint matrix
    m_constraintsMatrix(0, 0) = 1;
    m_constraintsMatrix(1, 1) = 1;
    m_constraintsMatrix(0, 3) = 1;
    m_constraintsMatrix(1, 4) = 1;
    m_constraintsMatrix(6, 2) = 1;

    m_hessianMatrix.resize(m_inputSize, m_inputSize);
    m_solution.resize(m_inputSize);

    // qpoases
    m_QPSolver_qpOASES = std::make_unique<qpOASES::SQProblem>(inputSize,
                                                              m_numberOfConstraints);

    m_QPSolver_qpOASES->setPrintLevel(qpOASES::PL_LOW);
    m_isFirstTime = true;
}

bool QPSolver::setHessianMatrix(const iDynTree::Vector2& zmpWeight, const iDynTree::Vector2& dcmOffsetWeight, const double& sigmaWeight)
{
    m_hessianMatrix(0,0) = zmpWeight(0);
    m_hessianMatrix(1,1) = zmpWeight(1);

    m_hessianMatrix(2,2) = sigmaWeight;

    m_hessianMatrix(3,3) = dcmOffsetWeight(0);
    m_hessianMatrix(4,4) = dcmOffsetWeight(1);

    return true;
}

bool QPSolver::setGradientVector(const iDynTree::Vector2& zmpWeight, const iDynTree::Vector2& dcmOffsetWeight, const double& sigmaWeight,
                                 const iDynTree::Vector2& zmpNominal, const iDynTree::Vector2& dcmOffsetNominal, const double& sigmaNominal)
{
    iDynTree::toEigen(m_gradient).segment(0, 2) = -(iDynTree::toEigen(zmpWeight).asDiagonal() * iDynTree::toEigen(zmpNominal));
    m_gradient(2) = -sigmaWeight * sigmaNominal;
    iDynTree::toEigen(m_gradient).segment(3, 2)  = -(iDynTree::toEigen(dcmOffsetWeight).asDiagonal() * iDynTree::toEigen(dcmOffsetNominal));

    if(m_QPSolver->isInitialized())
    {
        if(!m_QPSolver->updateGradient(iDynTree::toEigen(m_gradient)))
        {
            yError()<<"[QPSolver::setGradientVector]:unable to update the Gradient Vector";
            return false;
        }
    }
    else
    {
        if(!m_QPSolver->data()->setGradient(iDynTree::toEigen(m_gradient)))
        {
            yError()<<"[QPSolver::setGradientVector]:unable to set the Gradient Vector for the first time";
            return false;
        }
    }
    return true;
}

bool QPSolver::setConstraintsMatrix(const iDynTree::Vector2& currentDcmPosition, const iDynTree::Vector2& currentZmpPosition,
                                    const iDynTree::MatrixDynSize& convexHullMatrix)
{
    if(convexHullMatrix.rows() != 4 || convexHullMatrix.cols() != 2)
    {
        yError() << "QPSolver::setConstraintsMatrix the convex hull matrix size is strange " << convexHullMatrix.toString();
        return false;
    }

    iDynTree::Vector2 temp;
    iDynTree::toEigen(temp) = iDynTree::toEigen(currentZmpPosition) - iDynTree::toEigen(currentDcmPosition);

    m_constraintsMatrix(0, 2) = temp(0);
    m_constraintsMatrix(1, 2) = temp(1);
    m_constraintsMatrix(2, 0) = convexHullMatrix(0, 0);
    m_constraintsMatrix(2, 1) = convexHullMatrix(0, 1);
    m_constraintsMatrix(3, 0) = convexHullMatrix(1, 0);
    m_constraintsMatrix(3, 1) = convexHullMatrix(1, 1);
    m_constraintsMatrix(4, 0) = convexHullMatrix(2, 0);
    m_constraintsMatrix(4, 1) = convexHullMatrix(2, 1);
    m_constraintsMatrix(5, 0) = convexHullMatrix(3, 0);
    m_constraintsMatrix(5, 1) = convexHullMatrix(3, 1);

    return true;
}

bool QPSolver::setBoundsVectorOfConstraints(const iDynTree::Vector2& zmpPosition, const iDynTree::VectorDynSize& convexHullVector,
                                            const double& stepDuration, const double& stepDurationTollerance, const double& remainingSingleSupportDuration, const double& omega)
{
    if(convexHullVector.size() != 4)
    {
        yError() << "QPSolver::setConstraintsVector the convex hull vector size is strange " << convexHullVector.toString();
        return  false;
    }

    iDynTree::toEigen(m_upperBound).segment(0, 2) = iDynTree::toEigen(zmpPosition);
    iDynTree::toEigen(m_lowerBound).segment(0, 2) = iDynTree::toEigen(zmpPosition);
    iDynTree::toEigen(m_upperBound).segment(2, 4) = iDynTree::toEigen(convexHullVector);

    m_lowerBound(2) = -qpOASES::INFTY;
    m_lowerBound(3) = -qpOASES::INFTY;
    m_lowerBound(4) = -qpOASES::INFTY;
    m_lowerBound(5) = -qpOASES::INFTY;

    m_upperBound(6) = std::exp((stepDuration + stepDurationTollerance) * omega);
    m_lowerBound(6) = std::exp((stepDuration - std::min(stepDurationTollerance, remainingSingleSupportDuration)) * omega);

    return true;
}

bool QPSolver::isInitialized()
{
    return m_QPSolver->isInitialized();
}

bool QPSolver::initialize()
{
    return m_QPSolver->initSolver();
}

bool QPSolver::solve()
{
    MatrixXd constraintMatrix = MatrixXd(iDynTree::toEigen(m_constraintsMatrix));
    MatrixXd hessianMatrix = MatrixXd(iDynTree::toEigen(m_hessianMatrix));

    int nWSR = 100;
    if(!m_isFirstTime)
    {
        if(m_QPSolver_qpOASES->hotstart(hessianMatrix.data(), m_gradient.data(), constraintMatrix.data(),
                                        nullptr, nullptr,m_lowerBound.data(), m_upperBound.data(), nWSR, 0)
           != qpOASES::SUCCESSFUL_RETURN)
        {
            yError() << "[solve] Unable to solve the optimization problem.";
            return false;
        }
    }
    else
    {
        if(m_QPSolver_qpOASES->init(hessianMatrix.data(), m_gradient.data(), constraintMatrix.data(),
                                    nullptr, nullptr,m_lowerBound.data(), m_upperBound.data(), nWSR, 0)
           != qpOASES::SUCCESSFUL_RETURN)
        {
            yError() << "[solve] Unable to solve the optimization problem.";
            return false;
        }
        m_isFirstTime = false;
    }

    m_QPSolver_qpOASES->getPrimalSolution(m_solution.data());

    return true;
}

iDynTree::VectorDynSize QPSolver::getSolution()
{
    return m_solution;
}