//======================================================================== 
// Project: The Bin Observer
// ---------------------------------------------------------------------- 
// Package: The Bin Observer
// Authors: Vilas Kumar Chitrakaran
// Start Date: Mon Jun 27 2005
// Compiler: GNU C++
// Operating System: QNX 6.2.1 Momentics
// ----------------------------------------------------------------------  
// File: BinObserver.hpp
// Interface of the class BinObserver.
//========================================================================  
 

#ifndef INCLUDED_BinObserver_hpp
#define INCLUDED_BinObserver_hpp

#include "Vector.hpp"
#include "Adams3Integrator.hpp"

//======================================================================== 
// class BinObserver
// ---------------------------------------------------------------------- 
// \brief
// The BinObserver class interface. 
//
// This is the implementation of a variable structure velocity observer 
// (a.k.a the Bin Observer in CRB) from the paper:
// B. Xian, M.S. de Queiroz, D.M. Dawson, and M.L. McIntyre, "A 
// Discontinuous Output Feedback Controller and Velocity Observer for
// Nonlinear Mechanical Systems," Automatica, Vol. 40(4), pp. 695-700,
// April 2004. 
// 
// The observer is of the form:
// x_dot_estimate = p + k0 * x_error,
// p_dot = k1 * sgn(x_error) + k2 * x_error,
// where x_error = x - x_estimate, and x is the position measurement.
// 
// This estimator could be used for smooth estimation of the time 
// derivative (compared to numerical differentiation), given the signal 
// itself.
//
// Input: measured position (x)
// Output: estimated velocity. (x_dot_estimate)
// parameters: gains k0, k1 and k2.
//========================================================================  

template<int n>  // n is the length of the state vector x.
class BinObserver
{
 public:
  BinObserver();
   // The constructor. Initializes some internal variables.
   
  ~BinObserver(){};
   // Destructor does nothing
   
  void reset( Vector<n> &k0, Vector<n> &k1, Vector<n> &k2, double dt );
   // Call this function to initialize the observer.
   //  k0, k1, k2  constant gains.
   //  dt          sampling period.
    
  Vector<n> getNextDerivEstimate( Vector<n> &x );
   // Compute estimate for time derivative of the
   // measured signal for the next sample period.
   //  x  measured signal
   
 // ========== END OF INTERFACE ==========
			
 private:
  Vector<n> sgn( Vector<n> &x);
  Vector<n> d_k0, d_k1, d_k2;
  Vector<n> d_xEstimate;
  Adams3Integrator< Vector<n> > d_integrator01, d_integrator02;
  bool d_isInitialized;
};


//======================================================================== 
// BinObserver::BinObserver
//======================================================================== 
template<int n>
BinObserver<n>::BinObserver()
{
 d_k0 = 0;
 d_k1 = 0;
 d_k2 = 0;
 d_xEstimate = 0;
 d_isInitialized = false;
 d_integrator01.reset(d_xEstimate);
}

//======================================================================== 
// BinObserver::reset
//======================================================================== 
template<int n>
void BinObserver<n>::reset(Vector<n> &k0, Vector<n> &k1, Vector<n> &k2, double dt)
{
 d_k0 = k0;
 d_k1 = k1;
 d_k2 = k2;
 d_xEstimate = 0;
 
 d_integrator01.setSamplingPeriod(dt);
 d_integrator02.setSamplingPeriod(dt);
 d_integrator01.reset(d_xEstimate);
 
 d_isInitialized = false; // integrator02 is reset first time 
                          // getNextDerivEstimate() is called.
}

//======================================================================== 
// BinObserver::getNextDerivEstimate
//======================================================================== 
template<int n>
Vector<n> BinObserver<n>::getNextDerivEstimate( Vector<n> &x )
{
 Vector<n> xDotEstimate, xError, tmp0, tmp1, tmp2;
 
 if(!d_isInitialized)
 {
  d_xEstimate = x;
  d_integrator02.reset(d_xEstimate);
  d_isInitialized = true;
 }
 
 xError = x - d_xEstimate;
 
 tmp0 = sgn(xError);
 tmp1 = elementProduct(d_k1, tmp0) + elementProduct(d_k2, xError);
 tmp2 = elementProduct(d_k0, xError);
 xDotEstimate = d_integrator01.integrate(tmp1) + tmp2;
 d_xEstimate = d_integrator02.integrate(xDotEstimate);
 
 return xDotEstimate;
}

//======================================================================== 
// sgn
//======================================================================== 
template<int n>
Vector<n> BinObserver<n>::sgn( Vector<n> &x)
{
 Vector<n> s;
 double o = n + 1;
 for (int i = 1; i < o; i++)
  s(i) = tanh(1000 * x(i)); // Standard signum screws up
 return s;
}


#endif // INCLUDED_BinObserver_hpp

