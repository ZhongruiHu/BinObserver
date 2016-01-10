//======================================================================== 
// Project: The Bin Observer
// ---------------------------------------------------------------------- 
// Package: The Bin Observer
// Authors: Vilas Kumar Chitrakaran
// Start Date: Mon Jun 27 2005
// Compiler: GNU C++
// Operating System: QNX 6.2.1 Momentics
// ----------------------------------------------------------------------  
// File: BinObserver.t.cpp
// Example program for BinObserver
//========================================================================

#include "BinObserver.hpp"
#include <math.h>
#include <stdio.h>


int main()
{
 FILE *outfile;                    // File to store results
 Vector<2> position;               // some data
 Vector<2> velocity_numerical;     // observed derivative
 Vector<2> velocity_actual;        // actual derivative
 Vector<2> error;                  // estimation error
 double dt;                        // sampling period
 BinObserver<2> observer;          // observer
 Vector<2> k0, k1, k2;             // observer gains

 dt = 0.001;
 k0 = 5, 5;
 k1 = 1, 1;
 k2 = 1, 1;
 observer.reset(k0, k1, k2, dt);
 position = 0;
 
 outfile = fopen("BinObserver.dat", "w+");
 fprintf(outfile, "%s\n%s %s %s\n", "%Bin observer output file",
         "%position", "velocity_numerical", "velocity_actual" );
 for (int i = 0; i < 100000; i++)
 {
  velocity_actual = sin(2.0 * M_PI * i * dt); 
  position = (1.0/(2.0 * M_PI)) *(1 - cos(2.0 * M_PI * i * dt));
  
  // observe velocity
  velocity_numerical = observer.getNextDerivEstimate(position);

  // write the outputs to a file... 
  fprintf(outfile, "%f %f %f\n", velocity_numerical(1), 
          velocity_actual(1), error(1));
 }
 
 fclose(outfile);
 return(0);}
