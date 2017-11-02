
#include <iostream>


#include <maya/MFnPlugin.h>


#include "solverCmd.h"

MStatus initializePlugin(MObject obj)
{
	MFnPlugin pluginFn(obj, "Kyle Reed", "1.0");
	MStatus stat;
	cout.rdbuf(cerr.rdbuf()); //hack to get error messages out in Maya 2016.5
	cout << "-> Blendshape Solver Plugin Initialized" << endl;
	//fflush(stdout);
	//fflush(stderr);

	stat = pluginFn.registerCommand("blendshapeSolver", SolverCmd::creator);
	if (!stat)
		stat.perror("registerCommand failed");
	return stat;
}

MStatus uninitializePlugin(MObject obj)
{
	MFnPlugin pluginFn(obj);
	MStatus stat;
	stat = pluginFn.deregisterCommand("blendshapeSolver");
	if (!stat)
		stat.perror("deregisterCommand failed");
	return stat;
}