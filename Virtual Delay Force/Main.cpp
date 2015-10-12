//===========================================================================
/*
This file is part of the CHAI 3D visualization and haptics libraries.
Copyright (C) 2003-2009 by CHAI 3D. All rights reserved.

This library is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License("GPL") version 2
as published by the Free Software Foundation.

For using the CHAI 3D libraries with software that can not be combined
with the GNU GPL, and for taking advantage of the additional benefits
of our support services, please contact CHAI 3D about acquiring a
Professional Edition License.

\author    <http://www.chai3d.org>
\author    Francois Conti
\version   2.0.0 $Rev: 269 $
*/
//===========================================================================

#define GLEW_STATIC
#include "GL\glew.h"
//---------------------------------------------------------------------------
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <iomanip>
//---------------------------------------------------------------------------
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL
#define OVR_OS_WIN32
#include "GLFW\glfw3.h"
#include "GLFW\glfw3native.h"
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "GEL3D.h"
//---------------------------------------------------------------------------
#include "OVR.h"
#include "OVR_CAPI_GL.h"
#include "OVR_CAPI.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <chrono>
#include <random>       // std::default_random_engine
//---------------------------------------------------------------------------
using namespace std;
using namespace OVR;

//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a camera that renders the world in a window display
cCamera* camera;

// a haptic device
cGenericHapticDevice* hapticDevice;

// force scale factor
double deviceForceScale;

// scale factor between the device workspace and cursor workspace
double workspaceScaleFactor;

// has exited haptics simulation thread
bool simulationFinished = false;

//---------------------------------------------------------------------------
// GEL 3D
//---------------------------------------------------------------------------

// deformable world
cGELWorld* defWorld;

// object mesh
cGELMesh* defObject;

cGELMesh* ground;

// dynamic nodes
const int nodesPerSide = 8;
cGELSkeletonNode* nodes[nodesPerSide][nodesPerSide][nodesPerSide];

// device  model
cShapeSphere* device;
double deviceRadius;

// radius of the dynamic model sphere (GEM)
double radius;

// stiffness properties between the haptic device tool and the model (GEM)
const double listOfStiffness[5] = {3, 4, 5, 6, 7};
vector<double> stiffness(listOfStiffness, listOfStiffness + sizeof(listOfStiffness)/sizeof(listOfStiffness[0]) );
int caseNumber = 0;

// Is the visual shown or hidden
bool renderImage = true;

// Create the list for force
vector<cVector3d> forceList;

// Show the skeleton model of the object
bool showSkeletonModel = false;

// Input for participant's name
string name;

// Use delay force
bool useDelay = false;

// Add delay force
bool addForce = true;


//---------------------------------------------------------------------------
// OVR HMD
//---------------------------------------------------------------------------

ovrHmd				hmd;						// The handle of the headset
ovrHmdDesc			hmdDesc;
ovrEyeRenderDesc	eyeRenderDesc[2];			// Description of the VR
ovrVector3f			eyeOffsets[2];
ovrVector3f			cameraPosition;
GLFWwindow*			window;
Matrix4f			projectionMatrici[2];
GLuint				texture;
GLuint				framebuffer;
GLuint				depthbuffer;
ovrGLConfig			cfg;
double				lastTime;
int					nbFrames;
float				Yaw;				// Horizontal rotation of the player

vector<double> zPosition;
vector<double> zForce;
double stiffnessMax;
double level = -2.0;
ovrFrameTiming frameTiming;
int totalCase = 0;

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

class DataWithStiffness{
public:
	DataWithStiffness() {}

	DataWithStiffness(double stiffness, bool useDelay, bool renderImage)
		: Stiffness(stiffness), UseDelay(useDelay), RenderImage(renderImage) {}

	double stiffness() const { return Stiffness; }
	bool useDelay() const { return UseDelay; }
	bool renderImage() const { return RenderImage; }

	void Data(double stiffness, bool useDelay, bool renderImage)
	{
		Stiffness = stiffness;
		UseDelay = useDelay;
		RenderImage = renderImage;
	}

private:
	
	double Stiffness;
	
	bool RenderImage;

	bool UseDelay;
};

vector<DataWithStiffness> result;
void WriteResult(vector<DataWithStiffness> data)
{
	ofstream writeToFile("Result_" + name + ".txt", ios::trunc);
	if (writeToFile.is_open())
	{
		writeToFile << "Name: " << name << endl;
		writeToFile << "---------------------------------------------------------------------------" << endl;
		for (int i = 0; i < data.size(); i++)
		{
			writeToFile << "Case " << i + 1 << endl;
			writeToFile << "Stiffness: " << data[i].stiffness() << endl;
			if (data[i].renderImage())
				writeToFile << "Visual shown" << endl;
			else
				writeToFile << "Visual hidden" << endl;
			if (data[i].useDelay())
				writeToFile << "With delay" << endl;
			else
				writeToFile << "No delay" << endl;
			writeToFile << "---------------------------------------------------------------------------" << endl << endl;
		}
		writeToFile.close();
	}
}

void CreateNewData(bool delay)
{
	DataWithStiffness newData;
	newData.Data(stiffness[caseNumber], delay, renderImage);
	result.push_back(newData);
}

void NewStateMessage()
{
	cout << endl << endl << "State no. " << totalCase + 1 << endl;
	if(renderImage)
		cout << "Visual is shown" << endl;
	else
		cout << "Visual is hidden" << endl;
	if(useDelay)
		cout << "With delay" << endl;
	else
		cout << "No delay" << endl;
	cout << "Stiffness = " << stiffness[caseNumber] << endl << endl;

	cout << "Press H to show/hide visual" << endl;
	cout << "Press S to show/hide skeleton" << endl;
	cout << "Press D to add/dismiss delay" << endl << endl;
	
	cout << "Press 1 if the object is softer without delay." << endl;
	cout << "Press 2 if the object is softer with delay." << endl << endl;
}

void StartNextState()
{
	if(renderImage)
	{
		renderImage = false;
		defObject->setShowEnabled(false, true);
		device->setShowEnabled(false, true);
	}
	else
	{
		if(caseNumber == stiffness.size() - 1)
		{
			caseNumber = 0;

			// Shuffle the stiffness list
			unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
			shuffle (stiffness.begin(), stiffness.end(), default_random_engine(seed));
		}
		else
			caseNumber++;

		renderImage = true;
		defObject->setShowEnabled(true, true);
		device->setShowEnabled(true, true);
	}

	useDelay = false;
	if(forceList.size()>0)
		forceList.clear();
	addForce = true;

	NewStateMessage();
}

void WriteForcePosition()
{
	ofstream writeToFile("ForcePosition_" + to_string(caseNumber) + ".txt", ios::trunc);
	if (writeToFile.is_open())
	{
		for(int i = 0; i<zForce.size(); i++)
		{
			writeToFile << fixed << setprecision(6) << zPosition[i] << "\t";
			writeToFile << fixed << setprecision(6) << zForce[i] << endl;
		}
		writeToFile.close();
	}
}

void FinishedCurrentState(bool delay)
{
		CreateNewData(delay);
		totalCase++;
		
		if (totalCase == (int)stiffness.size() * 6 * 2)
		{
			WriteResult(result);

			cout<< "Thank you for your participation!" << endl;

			glfwSetWindowShouldClose(window, GL_TRUE);
		}
		else
			StartNextState();
}

// callback when a keyboard key is pressed
static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	if ((key == 's' && action == GLFW_PRESS) || (key == 'S' && action == GLFW_PRESS))
	{
		showSkeletonModel = !showSkeletonModel;
		defObject->m_showSkeletonModel = showSkeletonModel;	// show skeleton
		if (showSkeletonModel)
			cout << "Skeleton is shown" << endl << endl;
		else
			cout << "Skeleton is hidden" << endl << endl;
	}

	if ((key == 'd' && action == GLFW_PRESS) || (key == 'D' && action == GLFW_PRESS))
	{
		useDelay = !useDelay;
		if (useDelay)
		{
			addForce=false;
			lastTime = glfwGetTime();
			cout << "Use Delay 10 ms." << endl << endl;
		}
		else
		{
			forceList.clear();
			addForce=true;
			cout << "No delay is used." << endl << endl;
		}
	}

	if ((key == 'h' && action == GLFW_PRESS) || (key == 'H' && action == GLFW_PRESS))
	{
		renderImage = !renderImage;
		if (renderImage)
		{
			defObject->setShowEnabled(true, true);
			device->setShowEnabled(true, true);
			cout << "Visual is shown" << endl << endl;
		}
		else
		{
			defObject->setShowEnabled(false, true);
			device->setShowEnabled(false, true);
			cout << "Visual is hidden" << endl << endl;
		}
	}

	if ((key == '1' && action == GLFW_PRESS))
	{
		FinishedCurrentState(true);
	}

	if ((key == '2' && action == GLFW_PRESS))
	{
		FinishedCurrentState(false);
	}
}

//---------------------------------------------------------------------------

// function called before exiting the application
void close(void)
{
	//WriteForcePosition();
	// stop the simulation
	//simulationRunning = false;

	// wait for graphics and haptics loop to terminate
	while (!simulationFinished)
		cSleepMs(100);

	// close haptic device
	hapticDevice->close();
	glDeleteRenderbuffers(1, &depthbuffer);
	glDeleteTextures(1, &texture);
	glDeleteFramebuffers(1, &framebuffer);

	ovrHmd_Destroy(hmd);
	ovr_Shutdown();

	glfwDestroyWindow(window);
	glfwTerminate();
}

//---------------------------------------------------------------------------

void PrintHapticPosition(cVector3d pos)
{
#define SHOW(a) std::cout<< #a << ": " << (a) << std::endl;
	SHOW(pos);
	cout << "X: " << pos.x
		<< " Y: " << pos.y
		<< " Z: " << pos.z << endl;
}

cVector3d ComputeForce(const cVector3d& hapticPos, double deviceRadius, const cVector3d& nodePos, double radius, double stiffness)
{

	// compute the reaction forces between the tool and the ith sphere.
	cVector3d force;
	force.zero();
	cVector3d vSphereCursor = (hapticPos - nodePos);

	// check if both objects are intersecting
	if (vSphereCursor.length() < 0.0000001)
	{
		return (force);
	}

	if (vSphereCursor.length() > (deviceRadius + radius))
	{
		return (force);
	}

	// compute penetration distance between tool and surface of sphere
	double penetrationDistance = (deviceRadius + radius) - vSphereCursor.length();
	cVector3d forceDirection = cNormalize(vSphereCursor);
	forceDirection.z = fabs(forceDirection.z);
	//forceDirection[3]=fabs(forceDirection[3]);
	force = cMul(penetrationDistance * stiffness, forceDirection);

	// return result
	return (force);
}

void SendForcesToHaptic()
{	
	if(addForce)
	{
		hapticDevice->setForce(forceList[0]);
		if(forceList.size()>0)
			forceList.erase(forceList.begin());
	}
	else
	{
		double currentTime = glfwGetTime();
		if (currentTime - lastTime >= 0.01)
				addForce = true;		
	}
}

cVector3d CalculateReactionForce(cVector3d pos, cGELSkeletonNode* nodes[nodesPerSide][nodesPerSide], int xNode, int yNode)
{
	cVector3d nodePos = nodes[xNode][yNode]->m_pos;
	cVector3d f = ComputeForce(pos, deviceRadius, nodePos, radius, stiffness[caseNumber]);
	cVector3d tmpfrc = cNegate(f);
	nodes[xNode][yNode]->setExternalForce(tmpfrc);
	//force.add(f);

	return f;
}

// main haptics loop
void updateHaptics(void)
{
	// simulation clock
	cPrecisionClock simClock;
	
	// reset clock
	simClock.reset();
	
	// main haptic simulation loop
	while (!glfwWindowShouldClose(window))
	{
		// stop the simulation clock
		simClock.stop();

		// read the time increment in seconds
		double timeInterval = simClock.getCurrentTimeSeconds();
		if (timeInterval > 0.001) { timeInterval = 0.001; }

		// restart the simulation clock
		simClock.reset();
		simClock.start();

		// read position from haptic device
		cVector3d pos;
		hapticDevice->getPosition(pos);
		pos.mul(workspaceScaleFactor);
		device->setPos(pos);
		
		// init temp variable
		cVector3d force;
		force.zero();

		// compute reaction forces
		list<cGELMesh*>::iterator i;

		for(i = defWorld->m_gelMeshes.begin(); i != defWorld->m_gelMeshes.end(); ++i)
        {
			cGELMesh *nextItem = *i;

            if (nextItem->m_useMassParticleModel)
            {
                /*int numVertices = nextItem->m_gelVertices.size();
                for (int i=0; i<numVertices; i++)
                {
					int numVertices = nextItem->m_gelVertices.size();
					for (int i=0; i<numVertices; i++)
					{
						cVector3d nodePos = nextItem->m_gelVertices[i].m_massParticle->m_pos;
						cVector3d f = ComputeForce(pos, deviceRadius, nodePos, radius, 200);
						if (f.lengthsq() > 0)
						{
							cVector3d tmpfrc = cNegate(f);
							nextItem->m_gelVertices[i].m_massParticle->setExternalForce(tmpfrc);
						}
						force.add(cMul(1.0, f));
					}
				}*/
            }
			else
			{
				
				 // compute reaction forces
				for (int z=0; z<nodesPerSide; z++)
					{ 
					for (int y=0; y<nodesPerSide; y++)
						{
							for (int x=0; x<nodesPerSide; x++)
							{
							   cVector3d nodePos = nodes[x][y][z]->m_pos;
							   cVector3d f;
							   /*if(x == 1 || x == widthNodes - 2 || y == 1 || y == heightNodes - 2)
								   f = ComputeForce(pos, deviceRadius, nodePos, radius, 8);
							   else if(z == 0)
								   f = ComputeForce(pos, deviceRadius, nodePos, radius, 15);
							   else*/
								   f = ComputeForce(pos, deviceRadius, nodePos, radius, stiffness[caseNumber]);
							   cVector3d tmpfrc = cNegate(f);
							   nodes[x][y][z]->setExternalForce(tmpfrc);
							   force.add(f);
						   }
					}
				}
			}
		}

		defWorld->updateDynamics(timeInterval);

		// scale force
		force.mul(deviceForceScale);		

		/*double currentPosition = nodes[widthNodes / 2][widthNodes / 2]->m_pos.z;
		zPosition.push_back(currentPosition);
		zForce.push_back(force.z);*/

		// add force to the list of force
		forceList.push_back(force);
		SendForcesToHaptic();
	}

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------

void BeginFrame(int frameIndex, ovrPosef eyePoses[2])
{
	ovrHmd_BeginFrame(hmd, frameIndex);

	int width, height;
	float ratio;

	// Get eye poses for both the left and the right eye. g_EyePoses contains all Rift information: orientation, 
	// positional tracking and the IPD in the form of the input variable g_EyeOffsets.
	ovrHmd_GetEyePoses(hmd, frameIndex, eyeOffsets, eyePoses, NULL);

	// Bind the FBO...
	glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

	// Clear...
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glfwGetFramebufferSize(window, &width, &height);
	ratio = width / (float)height;
}

void DrawScene(ovrGLTexture eyeTexture[2], ovrPosef eyePoses[2])
{
	const int STEREO_RENDERING[2] = { CHAI_STEREO_RIGHT, CHAI_STEREO_LEFT };

	for (int eyeIndex = 0; eyeIndex < ovrEye_Count; eyeIndex++)
	{
		ovrEyeType eye = hmd->EyeRenderOrder[eyeIndex];
		eyePoses[eye].Position.x += Yaw;

		glViewport(eyeTexture[eye].OGL.Header.RenderViewport.Pos.x, eyeTexture[eye].OGL.Header.RenderViewport.Pos.y,
			eyeTexture[eye].OGL.Header.RenderViewport.Size.w, eyeTexture[eye].OGL.Header.RenderViewport.Size.h);
		
		camera->renderView(eyeTexture[eye].OGL.Header.RenderViewport.Size.w, eyeTexture[eye].OGL.Header.RenderViewport.Size.h,
			projectionMatrici[eye], eyePoses[eye], cameraPosition, STEREO_RENDERING[eye]);
	}
}

void EndFrame(ovrGLTexture(&eyeTexture)[2], int &frameIndex, ovrPosef eyePoses[2])
{
	//glfwSwapBuffers(window);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	ovrHmd_EndFrame(hmd, eyePoses, &eyeTexture[0].Texture);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); // Avoid OpenGL state leak in ovrHmd_EndFrame...
	glBindBuffer(GL_ARRAY_BUFFER, 0); // Avoid OpenGL state leak in ovrHmd_EndFrame...

	frameIndex++;
	glfwPollEvents();
}

void SetCameraPosition()
{
	// position and oriente the camera
	camera->set(cVector3d(cameraPosition.x, cameraPosition.y, cameraPosition.z),	// camera position (eye)
		cVector3d(0.0, 0.0, 0.0),			// lookat position (target)
		cVector3d(0.0, 0.0, 1.0));			// direction of the "up" vector
}

void UpdatePosition()
{
		ovrTrackingState ts = ovrHmd_GetTrackingState(hmd, ovr_GetTimeInSeconds());
		if (ts.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked)) 
		{
			ovrVector3f pose = ts.HeadPose.ThePose.Position;
			cameraPosition.x = -pose.x * 10;
			cameraPosition.y = -pose.y * 10;
			cameraPosition.z = -5.0 - pose.z*10;
			SetCameraPosition();
		}       
}

// main graphics callback
void updateGraphics(cThread* hapticsThread, ovrGLTexture(&eyeTexture)[2])
{
	ovrHSWDisplayState hswDisplayState;
	ovrHmd_GetHSWDisplayState(hmd, &hswDisplayState);
	if (hswDisplayState.Displayed)
		ovrHmd_DismissHSWDisplay(hmd);

	// starts the main haptics rendering loop
	hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

	int frameIndex = 0;
	ovrPosef eyePoses[2];

	while (!glfwWindowShouldClose(window))
	{
		// update mesh of deformable model
		defWorld->updateSkins();
		//ground->computeAllNormals(true);
		defObject->computeAllNormals(true);
		

		//CalculateFPS();
		BeginFrame(frameIndex, eyePoses);
		UpdatePosition();
		DrawScene(eyeTexture, eyePoses);
		EndFrame(eyeTexture, frameIndex, eyePoses);
	}
}
//---------------------------------------------------------------------------

void CreateCamera(cWorld* world)
{
	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	SetCameraPosition();	

	// set the near and far clipping planes of the camera
	// anything in front/behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 100.0);

	// enable higher rendering quality because we are displaying transparent objects
	camera->enableMultipassTransparency(true);

	// set stereo eye separation and focal length
	float ipd = eyeRenderDesc[ovrEye_Left].HmdToEyeViewOffset.x * -1.0;
	camera->setStereoEyeSeparation(ipd);
	camera->setStereoFocalLength(3.0);
}

void CreateLightSource(cWorld* world)
{
	// create a light source and attach it to the camera
	cLight* light = new cLight(world);
	camera->addChild(light);	// attach light to camera
	light->setEnabled(true);	// enable light source
	light->setPos(cVector3d(2.0, 0.5, 1.0));	// position the light source
	light->setDir(cVector3d(-2.0, 0.5, 1.0));	// define the direction of the light beam

	//return light;
}

void CreateHaptics(cWorld* world)
{
	//-----------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//-----------------------------------------------------------------------

	// create a haptic device handler
	cHapticDeviceHandler* handler = new cHapticDeviceHandler();

	// get access to the first available haptic device
	handler->getDevice(hapticDevice, 0);
	
	// retrieve information about the current haptic device
	cHapticDeviceInfo info;
	if (hapticDevice)
	{
		hapticDevice->open();
		info = hapticDevice->getSpecifications();
	}

	// desired workspace radius of the cursor
	double cursorWorkspaceRadius = 0.8;

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	workspaceScaleFactor = cursorWorkspaceRadius / info.m_workspaceRadius;

	// define a scale factor between the force perceived at the cursor and the
	// forces actually sent to the haptic device
	deviceForceScale = 0.1 * info.m_maxForce;

	// define a maximum stiffness that can be handled by the current
    // haptic device. The value is scaled to take into account the
    // workspace scale factor
    stiffnessMax = info.m_maxForceStiffness / workspaceScaleFactor;

	// create a large sphere that represents the haptic device
	deviceRadius = 0.1;
	device = new cShapeSphere(deviceRadius);
	world->addChild(device);
	device->m_material.m_ambient.set(0.4, 0.4, 0.4, 0.7);
	device->m_material.m_diffuse.set(0.7, 0.7, 0.7, 0.7);
	device->m_material.m_specular.set(1.0, 1.0, 1.0, 0.7);
	device->m_material.setShininess(100);
	//return handler;
}

cVector3d CreateNewVertex(const double xStart, const double yStart, const double zStart, const double xNext, const double yNext, const double zNext)
{
	cVector3d newVertex(xStart + xNext, yStart + yNext, zStart + zNext);
	return newVertex;
}

//---------------------------------------------------------------------------

void CreateCube(const double size)
{
	const int cubeSide = 12;						// Number of cube sides times 2: 6*2 = 12
	const int verticesPerSide = 81;					// Number of vertices based on square of the number of nodes on each side + 1. If nodesPerSide = 8, verticesPerSide = (8+1)^2 = 81
	int vertices[cubeSide][verticesPerSide];
	int verticesHV[6][verticesPerSide][verticesPerSide];
	int vertexNo;
	int vertexH = 0;
	int vertexV = 0;
	double staticPoint = size - size * 2 / nodesPerSide;
	// face -x	
	vertexNo = 0;
	vertexH = 0;
	vertexV = 0;
	for(double i = -size; i <= size; i += size * 2 / nodesPerSide)
	{
		for(double j = size; j >= -size; j -= size * 2 / nodesPerSide)
		{
			vertices[0][vertexNo] = defObject->newVertex(-size, i, j);
			vertices[6][vertexNo] = defObject->newVertex(-size, j, i);
			vertexNo++;

			verticesHV[0][vertexH][vertexV] = defObject->newVertex(-size, j, i);
			vertexV++;
		}
		vertexV = 0;
		vertexH++;
	}

	// face +x	
	vertexNo = 0;
	vertexH = 0;
	vertexV = 0;
	for(double i = -size; i <= size; i += size * 2 / nodesPerSide)
	{
		for(double j = -size; j <= size; j += size * 2 / nodesPerSide)
		{
			vertices[1][vertexNo] = defObject->newVertex(size, i, j);
			vertices[7][vertexNo] = defObject->newVertex(size, j, i);
			vertexNo++;

			verticesHV[1][vertexH][vertexV] = defObject->newVertex(size, j, i);
			vertexV++;
		}
		vertexV = 0;
		vertexH++;
	}

	// face -y	
	vertexNo = 0;
	vertexH = 0;
	vertexV = 0;
	for(double i = -size; i <= size; i += size * 2 / nodesPerSide)
	{
		for(double j = -size; j <= size; j += size * 2 / nodesPerSide)
		{
			vertices[2][vertexNo] = defObject->newVertex(i, -size, j);
			vertices[8][vertexNo] = defObject->newVertex(j, -size, i);
			vertexNo++;

			verticesHV[2][vertexH][vertexV] = defObject->newVertex(j+(size * 2 / nodesPerSide), -size, i);
			vertexV++;
		}
		vertexV = 0;
		vertexH++;
	}

	// face +y	
	vertexNo = 0;
	vertexH = 0;
	vertexV = 0;
	for(double i = -size; i <= size; i += size * 2 / nodesPerSide)
	{
		for(double j = size; j >= -size; j -= size * 2 / nodesPerSide)
		{
			vertices[3][vertexNo] = defObject->newVertex(i, size, j);
			vertices[9][vertexNo] = defObject->newVertex(j, size, i);
			vertexNo++;

			verticesHV[3][vertexH][vertexV] = defObject->newVertex(j, size-(size * 2 / nodesPerSide), i);
			vertexV++;
		}
		vertexV = 0;
		vertexH++;
	}

	// face -z
	vertexNo = 0;
	vertexH = 0;
	vertexV = 0;
	for(double i = -size; i <= size; i += size * 2 / nodesPerSide)
	{
		for(double j = -size; j <= size; j += size * 2 / nodesPerSide)
		{
			vertices[4][vertexNo] = defObject->newVertex(j, i, -size);
			vertices[10][vertexNo] = defObject->newVertex(i, j, -size);
			vertexNo++;

			verticesHV[4][vertexH][vertexV] = defObject->newVertex(i, j, -size);
			vertexV++;
		}
		vertexV = 0;
		vertexH++;
	}

	// face +z
	vertexNo = 0;
	vertexH = 0;
	vertexV = 0;
	for(double i = size; i >= -size; i -= size * 2 / nodesPerSide)
	{
		for(double j = -size; j <= size; j += size * 2 / nodesPerSide)
		{
			vertices[5][vertexNo] = defObject->newVertex(j, i, size);
			vertices[11][vertexNo] = defObject->newVertex(i, j, size);
			vertexNo++;

			verticesHV[5][vertexH][vertexV] = defObject->newVertex(i, j, size-(size * 2 / nodesPerSide));
			vertexV++;
		}
		vertexV = 0;
		vertexH++;
	}

	for(int i = 0; i < 6; i++)
		{
		for(int j = 0; j < nodesPerSide - 1; j++)
		{
			for(int k = 0; k < nodesPerSide - 1; k++)
			{
				defObject->newTriangle(verticesHV[i][j][k], verticesHV[i][j][k+1], verticesHV[i][j+1][k]);
				defObject->newTriangle( verticesHV[i][j+1][k],verticesHV[i][j][k+1], verticesHV[i][j+1][k+1]);
			}
		}
	}

	// create triangles
	/*for (int i = 0; i <6; i++)
	{
		for(int j = 0; j < verticesPerSide - nodesPerSide - 2; j++)
		{
			defObject->newTriangle(vertices[i][j], vertices[i][j + 1], vertices[i][j + nodesPerSide + 2]);
			defObject->newTriangle(vertices[i][j], vertices[i][j + nodesPerSide + 2], vertices[i][j + nodesPerSide + 1]);
		}
	}*/

	// set material properties to light gray
	defObject->m_material.m_ambient.set(0.5f, 0.5f, 0.5f, 1.0f);
	defObject->m_material.m_diffuse.set(0.7f, 0.7f, 0.7f, 1.0f);
	defObject->m_material.m_specular.set(1.0f, 1.0f, 1.0f, 1.0f);
	defObject->m_material.m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);

	// compute normals
	defObject->computeAllNormals();

	//defObject->setShowNormals(true);

	//defObject->setTransparencyLevel(0.0);
}

void CreateNodes(double additionalspacecube, double spacingSpheres)
{
	double defaultRadius = 0.1;		
	double startingPoint = spacingSpheres * nodesPerSide / (-2.0) - additionalspacecube;

	// setup default values for nodes
	cGELSkeletonNode::default_radius = defaultRadius * 5.0;
	cGELSkeletonNode::default_kDampingPos = 3.0;
	cGELSkeletonNode::default_kDampingRot = 3.0;
	cGELSkeletonNode::default_mass = 0.0008;  // [kg]
	cGELSkeletonNode::default_showFrame = true;
	cGELSkeletonNode::default_color.set(1.0, 0.6, 0.6);
	cGELSkeletonNode::default_useGravity = false;
	cGELSkeletonNode::default_gravity.set(0.00, 0.00, -1.00);

	radius = defaultRadius;

	for (int z = 0; z<nodesPerSide; z++)
	{
		for (int y = 0; y<nodesPerSide; y++)
		{
			for (int x = 0; x<nodesPerSide; x++)
			{
				cGELSkeletonNode* newNode = new cGELSkeletonNode();
				nodes[x][y][z] = newNode;
				defObject->m_nodes.push_front(newNode);

				double xPos = startingPoint + spacingSpheres * (double)x + 0.11;
				double yPos = startingPoint + spacingSpheres * (double)y - 0.11;
				double zPos = startingPoint + spacingSpheres * (double)z - 1.3;
				newNode->m_pos.set(xPos, yPos, zPos);

				if (z==0)
						nodes[x][y][z]->m_fixed = true;
			}
		}
	}
}

void CreateTextureMaterial()
{
	cMaterial mat;
	mat.m_ambient.set(239.0/255.0, 31.0/255.0, 31.0/255.0);
	mat.m_diffuse.set(239.0/255.0, 31.0/255.0, 31.0/255.0);
	mat.setShininess(100);
	defObject->setMaterial(mat, true);

	// let's create a some environment mapping
	/*cTexture2D* imTexture = new cTexture2D();
	bool fileload = imTexture->loadFromFile("../resources/images/shadow.bmp");
	if (!fileload)
	{
		cout << "Error - Texture image failed to load correctly.\n";
		close();
	}*/

	/*imTexture->setEnvironmentMode(GL_DECAL);
	imTexture->setSphericalMappingEnabled(true);
	defObject->setTexture(imTexture, true);*/
	defObject->setTransparencyLevel(1.0, true);
	//defObject->setUseTexture(true, true);
}

void CreateLinks()
{
	// setup default values for links
	cGELSkeletonLink::default_kSpringElongation = 10.0; // [N/m]
	cGELSkeletonLink::default_kSpringFlexion = 0.05;   // [Nm/RAD]
	cGELSkeletonLink::default_kSpringTorsion = 0.01;   // [Nm/RAD]
	cGELSkeletonLink::default_color.set(0.2, 0.2, 1.0);

	for (int z = 1; z<nodesPerSide-1; z++)
	{
		for (int y = 1; y<nodesPerSide-1; y++)
		{
			for (int x = 1; x<nodesPerSide-1; x++)
			{
				for (int zp = -1; zp<2; zp++)
				{
					for (int yp = -1; yp<2; yp++)
					{
						for (int xp = -1; xp<2; xp++)
						{
							if ((zp!=0)&&(xp!=0)&&(yp!=0))
							{
								cGELSkeletonLink* newLinkX0 = new cGELSkeletonLink(nodes[x][y][z], nodes[x+xp][y+yp][z+zp]);
								defObject->m_links.push_front(newLinkX0);
							}
						}
					}
				}
			}
		}
	}
}

void CreateVirtualSpringObject(cWorld* world)
{
	double additionalspacecube = -0.1;
	double spacingSpheres = 0.1;

	// create a deformable mesh
	defObject = new cGELMesh(world);
	CreateCube( spacingSpheres * (nodesPerSide - 1) + additionalspacecube * 2);
	defWorld->m_gelMeshes.push_front(defObject);
	
	// set some material color on the object
	CreateTextureMaterial();

	//// build dynamic vertices
	defObject->buildVertices();
	defObject->m_useSkeletonModel = true;

	CreateNodes(additionalspacecube, spacingSpheres);
	CreateLinks();

	// connect skin (mesh) to skeleton (GEM)
	defObject->connectVerticesToSkeleton(false);

	defObject->translate(0.0, 0.0, -1.3);

	// show/hide underlying dynamic skeleton model
	defObject->m_showSkeletonModel = false;
}

void CreateTable(cWorld* world)
{
	//-----------------------------------------------------------------------
    // COMPOSE THE TABLE
    //-----------------------------------------------------------------------
    ground = new cGELMesh(world);
    ground->m_useMassParticleModel = true;
    defWorld->m_gelMeshes.push_front(ground);
	
    cGELMassParticle::default_mass = 100.0;
    cGELMassParticle::default_kDampingPos = 0.0;
    cGELMassParticle::default_gravity.set(0,0,0);

    int u,v;
    int RESOLUTION = 15;
    double SIZE = 10.0;
    for (v=0; v<RESOLUTION; v++)
    {
        for (u=0; u<RESOLUTION; u++)
        {
            double px, py, tu, tv;

            // compute the position of the vertex
            px = SIZE / (double)RESOLUTION * (double)u - (SIZE/2.0);
            py = SIZE / (double)RESOLUTION * (double)v - (SIZE/2.0);

            // create new vertex
            unsigned int index = ground->newVertex(px, py, level);
            cVertex* vertex = ground->getVertex(index);

            // compute texture coordinate
            tu = (double)u / (double)RESOLUTION;
            tv = (double)v / (double)RESOLUTION;
            vertex->setTexCoord(tu, tv);
            vertex->setColor(cColorf(1.0, 0.0, 0.1));
        }
    }

    ground->buildVertices();

    for (v=0; v<RESOLUTION; v++)
    {
        for (u=0; u<RESOLUTION; u++)
        {
            if ((u == 0) || (v == 0) || (u == (RESOLUTION-1)) || (v == (RESOLUTION-1)))
            {
                // create new vertex
                unsigned int index = ((v + 0) * RESOLUTION) + (u + 0);
                ground->m_gelVertices[index].m_massParticle->m_fixed = true;
            }
        }
    }

    cGELLinearSpring::default_kSpringElongation = 0.0; // [N/m]

    // Create a triangle based map using the above pixels
     for (v=0; v<(RESOLUTION-1); v++)
    {
        for (u=0; u<(RESOLUTION-1); u++)
        {
            // get the indexing numbers of the next four vertices
            unsigned int index00 = ((v + 0) * RESOLUTION) + (u + 0);
            unsigned int index01 = ((v + 0) * RESOLUTION) + (u + 1);
            unsigned int index10 = ((v + 1) * RESOLUTION) + (u + 0);
            unsigned int index11 = ((v + 1) * RESOLUTION) + (u + 1);

            // create two new triangles
            ground->newTriangle(index00, index01, index10);
            ground->newTriangle(index10, index01, index11);

            cGELMassParticle* m0 = ground->m_gelVertices[index00].m_massParticle;
            cGELMassParticle* m1 = ground->m_gelVertices[index01].m_massParticle;
            cGELMassParticle* m2 = ground->m_gelVertices[index10].m_massParticle;

			m0->m_fixed = true;
			m1->m_fixed = true;
			m2->m_fixed = true;

            cGELLinearSpring* spring0 = new cGELLinearSpring(m0, m1);
            cGELLinearSpring* spring1 = new cGELLinearSpring(m0, m2);
            ground->m_linearSprings.push_back(spring0);
            ground->m_linearSprings.push_back(spring1);
        }
    }

    double transparencyLevel = 0.7;
    ground->setUseMaterial(true);
    ground->m_material.m_ambient.set(0.6, 0.6, 0.6, transparencyLevel);
    ground->m_material.m_diffuse.set(0.8, 0.8, 0.8, transparencyLevel);
    ground->m_material.m_specular.set(0.9, 0.9, 0.9, transparencyLevel);
    ground->setTransparencyLevel(transparencyLevel);
    ground->setUseTransparency(true);
    cTexture2D* textureGround = new cTexture2D();
    ground->setTexture(textureGround);
    ground->setUseTexture(true, true);

    bool fileload = textureGround->loadFromFile("../resources/images/stone.bmp");
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly.\n";
		close();
    }
}

void ComposeVirtualScene(cWorld* world)
{
	//-----------------------------------------------------------------------
	// COMPOSE THE VIRTUAL SCENE
	//-----------------------------------------------------------------------

	// create a world which supports deformable object
	defWorld = new cGELWorld();
	world->addChild(defWorld);	

	CreateVirtualSpringObject(world);	
	CreateTable(world);

	// define the integration time constant of the dynamics model
	defWorld->m_integrationTime = 0.005;
}

// creates the world
cWorld* theWorld()
{
	//-----------------------------------------------------------------------
	// 3D - SCENEGRAPH
	//-----------------------------------------------------------------------

	// create a new world.
	cWorld* world = new cWorld();

	// set the background colour of the environment
	// the colour is defined by its (R,G,B) components.
	//world->setBackgroundColor(0.0, 0.0, 0.0);
	world->setBackgroundColor(255.0/255.0, 239.0/255.0, 239.0/255.0);

	CreateCamera(world);

	CreateLightSource(world);

	CreateHaptics(world);

	ComposeVirtualScene(world);

	return world;
}

// set window
void SetWindow(Sizei resolution, GLFWmonitor* monitor)
{
	window = glfwCreateWindow(resolution.w, resolution.h, "tes", monitor, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	ovrHmd_AttachToWindow(hmd, glfwGetWin32Window(window), NULL, NULL);
	glfwMakeContextCurrent(window);
}

void ErrorCallback(int error, const char* description)
{
	fputs(description, stderr);
}

void SetTexture(Sizei renderTargetSize)
{
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, renderTargetSize.w, renderTargetSize.h, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

void SetDepthbuffer(Sizei renderTargetSize)
{
	glGenRenderbuffers(1, &depthbuffer);
	glBindRenderbuffer(GL_RENDERBUFFER, depthbuffer);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, renderTargetSize.w, renderTargetSize.h);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthbuffer);
}

static void WindowSizeCallback(GLFWwindow* window, int width, int height)
{
	if (width>0 && height>0)
	{
		cfg.OGL.Header.BackBufferSize.w = width;
		cfg.OGL.Header.BackBufferSize.h = height;

		ovrBool configureResult = ovrHmd_ConfigureRendering(hmd, &cfg.Config, ovrDistortionCap_Chromatic | ovrDistortionCap_Vignette |
			ovrDistortionCap_TimeWarp | ovrDistortionCap_Overdrive, hmd->MaxEyeFov, eyeRenderDesc);
		glUseProgram(0); // Avoid OpenGL state leak in ovrHmd_ConfigureRendering...
		if (!configureResult)
		{
			cout << "Configure failed.\n";
			exit(EXIT_FAILURE);
		}
	}
}

Sizei SetResolution(GLFWmonitor* &monitor)
{
	Sizei resolution;

	// Check to see if the hmd in "Direct" or "Extended" mode
	bool directMode = ((hmd->HmdCaps & ovrHmdCap_ExtendDesktop) == 0);
	if (directMode)
	{
		std::cout << "Running in \"Direct\" mode" << endl;
		monitor = NULL;

		resolution.w = hmd->Resolution.w / 2;
		resolution.h = hmd->Resolution.h / 2;
	}
	else
	{
		std::cout << "Running in \"Extended Desktop\" mode" << endl;
		int countMonitor;
		GLFWmonitor** listMonitors = glfwGetMonitors(&countMonitor);

		switch (countMonitor)
		{
		case 0:
			std::cout << "No monitors found." << endl;
			exit(EXIT_FAILURE);
			break;
		case 1:
			std::cout << "Two monitors expected, found only one, using primary." << endl;
			monitor = glfwGetPrimaryMonitor();
			break;
		case 2:
			std::cout << "Two monitors found, using second monitor." << endl;
			monitor = listMonitors[1];
			break;
		default:
			std::cout << "More than two monitors found, using second monitor." << endl;
			monitor = listMonitors[1];
		}

		resolution.w = hmd->Resolution.w;
		resolution.h = hmd->Resolution.h;
	}

	return resolution;
}

void SetEyeRenderViewport(ovrRecti(&eyeRenderViewport)[2], ovrSizei eyeTextureSizes[2], Sizei renderTargetSize)
{
	eyeRenderViewport[ovrEye_Left].Pos = Vector2i(0, 0);
	eyeRenderViewport[ovrEye_Left].Size = eyeTextureSizes[ovrEye_Left];
	eyeRenderViewport[ovrEye_Right].Pos = Vector2i((renderTargetSize.w + 1) / 2, 0);
	eyeRenderViewport[ovrEye_Right].Size = eyeRenderViewport[0].Size;
}

void SetEyeTexture(ovrGLTexture(&eyeTexture)[2], ovrSizei eyeTextureSizes[2], Sizei renderTargetSize)
{
	ovrRecti eyeRenderViewport[2];
	SetEyeRenderViewport(eyeRenderViewport, eyeTextureSizes, renderTargetSize);

	eyeTexture[ovrEye_Left].OGL.Header.API = ovrRenderAPI_OpenGL;
	eyeTexture[ovrEye_Left].OGL.Header.TextureSize = renderTargetSize;
	eyeTexture[ovrEye_Left].OGL.Header.RenderViewport = eyeRenderViewport[ovrEye_Left];
	eyeTexture[ovrEye_Left].OGL.TexId = texture;

	eyeTexture[ovrEye_Right] = eyeTexture[ovrEye_Left];
	eyeTexture[ovrEye_Right].OGL.Header.RenderViewport = eyeRenderViewport[ovrEye_Right];
}

void SetConfig(Sizei resolution)
{
	cfg.OGL.Header.API = ovrRenderAPI_OpenGL;
	cfg.OGL.Header.BackBufferSize = Sizei(resolution.w, resolution.h);
	cfg.OGL.Header.Multisample = 1;
	cfg.OGL.Window = glfwGetWin32Window(window);
	cfg.OGL.DC = GetDC(cfg.OGL.Window);
}

static void SetStaticLightPositions(void)
{
	GLfloat l_Light0Position[] = { 3.0f, 4.0f, 2.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, l_Light0Position);

	GLfloat l_Light1Position[] = { -3.0f, -4.0f, 2.0f, 0.0f };
	glLightfv(GL_LIGHT1, GL_POSITION, l_Light1Position);
}

void PrintHeadPosition(ovrTrackingState state)
{
	cout << "Head Position:" << endl;
	cout << "X: " << state.HeadPose.ThePose.Position.x
		<< " Y: " << state.HeadPose.ThePose.Position.y
		<< " Z: " << state.HeadPose.ThePose.Position.z << endl;
}

void CheckFramebuffer()
{
	// Check if everything is OK...
	GLenum l_Check = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
	if (l_Check != GL_FRAMEBUFFER_COMPLETE)
	{
		cout << "There is a problem with the FBO." << endl;
		exit(EXIT_FAILURE);
	}
}

void UnbindBuffer()
{
	// Unbind...
	glBindRenderbuffer(GL_RENDERBUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void StartSensor()
{
	// Start the sensor which provides the Riftfs pose and motion...
	GLuint supportedSensorCaps = ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position;
	GLuint requiredTrackingCaps = 0;
	ovrBool trackingResult = ovrHmd_ConfigureTracking(hmd, supportedSensorCaps, requiredTrackingCaps);
	if (!trackingResult)
	{
		cout << "Could not start tracking..." << endl;
		exit(EXIT_FAILURE);
	}
}

void InitCamera()
{
	// Initial camera position...
	cameraPosition.x = 0.0f;
	cameraPosition.y = -0.5f;
	cameraPosition.z = -5.0f;
}

void InitGlew()
{
	GLenum glewErr = glewInit();
	if (glewInit() != GLEW_OK)
	{
		cerr << glewGetErrorString(glewErr) << endl;
	}
}

void InitHmd()
{
	//Initialize the Rift
	ovr_Initialize();
	hmd = ovrHmd_Create(0);

	//Start debug mode if the Rift isn't detected
	if (!hmd)
		hmd = ovrHmd_CreateDebug(ovrHmd_DK2);
}

// initialization
void Initialization(ovrGLTexture(&eyeTexture)[2])
{
	//-----------------------------------------------------------------------
	// INITIALIZATION
	//-----------------------------------------------------------------------

	InitHmd();

	if (!glfwInit())
		exit(EXIT_FAILURE);

	GLFWmonitor *monitor;
	Sizei resolution = SetResolution(monitor);

	glfwSetErrorCallback(ErrorCallback);

	SetWindow(resolution, monitor);

	InitGlew();

	ovrSizei eyeTextureSizes[2];
	eyeTextureSizes[ovrEye_Left] = ovrHmd_GetFovTextureSize(hmd, ovrEye_Left, hmd->DefaultEyeFov[ovrEye_Left], 1.0f);
	eyeTextureSizes[ovrEye_Right] = ovrHmd_GetFovTextureSize(hmd, ovrEye_Right, hmd->DefaultEyeFov[ovrEye_Right], 1.0f);

	Sizei renderTargetSize;
	renderTargetSize.w = eyeTextureSizes[ovrEye_Left].w + eyeTextureSizes[ovrEye_Right].w;
	renderTargetSize.h = (eyeTextureSizes[ovrEye_Left].h > eyeTextureSizes[ovrEye_Right].h ?
		eyeTextureSizes[ovrEye_Left].h : eyeTextureSizes[ovrEye_Right].h);

	glGenFramebuffers(1, &framebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

	// Create Texture
	SetTexture(renderTargetSize);

	// Create Depth Buffer
	SetDepthbuffer(renderTargetSize);

	// Set the texture as our colour attachment #0...
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture, 0);

	// Set the list of draw buffers...
	GLenum drawbuffers[1] = { GL_COLOR_ATTACHMENT0 };
	glDrawBuffers(1, drawbuffers); // "1" is the size of DrawBuffers

	CheckFramebuffer();

	UnbindBuffer();

	SetEyeTexture(eyeTexture, eyeTextureSizes, renderTargetSize);

	SetConfig(resolution);

	//ovrFovPort eyeFovPorts[2] = { hmd->DefaultEyeFov[0], hmd->DefaultEyeFov[1] };
	ovrHmd_ConfigureRendering(hmd, &cfg.Config, ovrDistortionCap_Chromatic | ovrDistortionCap_Vignette |
		ovrDistortionCap_TimeWarp | ovrDistortionCap_Overdrive, hmd->MaxEyeFov, eyeRenderDesc);
	glUseProgram(0); // Avoid OpenGL state leak in ovrHmd_ConfigureRendering...

	StartSensor();

	// Projection matrici for each eye will not change at runtime, we can set them here...
	projectionMatrici[ovrEye_Left] = ovrMatrix4f_Projection(eyeRenderDesc[ovrEye_Left].Fov, 0.3f, 100.0f, true);
	projectionMatrici[ovrEye_Right] = ovrMatrix4f_Projection(eyeRenderDesc[ovrEye_Right].Fov, 0.3f, 100.0f, true);

	// IPD offset values will not change at runtime, we can set them here...
	eyeOffsets[ovrEye_Left] = eyeRenderDesc[ovrEye_Left].HmdToEyeViewOffset;
	eyeOffsets[ovrEye_Right] = eyeRenderDesc[ovrEye_Right].HmdToEyeViewOffset;

	InitCamera();

	cWorld* world = theWorld();

	lastTime = glfwGetTime();
	nbFrames = 0;
	Yaw = 0;

	cVector3d pos;
	hapticDevice->getPosition(pos);
	pos.mul(workspaceScaleFactor);
	device->setPos(pos);
}

void OpeningMessage()
{
	cout << string(50, '\n');
	cout << "Virtual Delay Force" << endl << endl;
	NewStateMessage();
}

int main(int argc, char*argv[])
{
	cout << "Participant name = ";
	cin >> name;

	// create a thread for the main haptics
	cThread* hapticsThread = new cThread();
	ovrGLTexture eyeTexture[2];
	Initialization(eyeTexture);

	glfwSetKeyCallback(window, KeyCallback);
	glfwSetWindowSizeCallback(window, WindowSizeCallback);
	ovrHmd_RecenterPose(hmd);

	OpeningMessage();
	updateGraphics(hapticsThread, eyeTexture);

	close();


  return 0;
}
