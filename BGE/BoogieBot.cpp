#include "BoogieBot.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

using namespace BGE;

BoogieBot::BoogieBot(void)
{
}

BoogieBot::~BoogieBot(void)
{
}

bool BoogieBot::Initialise()
{
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	setGravity(glm::vec3(0, -30, 0));
	
	time = 0;

	force = 600;

	shared_ptr<PhysicsController> box1 = physicsFactory->CreateBox(5, 5, 5, glm::vec3(0, 7, 0), glm::quat());
	shared_ptr<PhysicsController> head = physicsFactory->CreateBox(2, 2, 2, glm::vec3(0, 12.7, 0), glm::quat());
	shared_ptr<PhysicsController> frontright = physicsFactory->CreateBox(1.5f, 3.8f, 1.5f, glm::vec3(2.5f, 7.5f, -2.5f), glm::quat());
	shared_ptr<PhysicsController> backleft = physicsFactory->CreateBox(1.5f, 3.8f, 1.5f, glm::vec3(-2.5f, 7.5f, 2.5f), glm::quat());
	shared_ptr<PhysicsController> backright = physicsFactory->CreateBox(1.5f, 3.8f, 1.5f, glm::vec3(-2.5f, 7.5f, -2.5f), glm::quat());
	shared_ptr<PhysicsController> frontleft = physicsFactory->CreateBox(1.5f, 3.8f, 1.5f, glm::vec3(2.5f, 7.5f, 2.5f), glm::quat());

	shared_ptr<PhysicsController> foot1 = physicsFactory->CreateBox(1.5f, 1.8f, 1.5f, glm::vec3(-2.5f, 0.5f, 2.5f), glm::quat());
	shared_ptr<PhysicsController> foot2 = physicsFactory->CreateBox(1.5f, 1.8f, 1.5f, glm::vec3(2.5f, 0.5f, 2.5f), glm::quat());
	shared_ptr<PhysicsController> foot3 = physicsFactory->CreateBox(1.5f, 1.8f, 1.5f, glm::vec3(2.5f, 0.5f, -2.5f), glm::quat());
	shared_ptr<PhysicsController> foot4 = physicsFactory->CreateBox(1.5f, 1.8f, 1.5f, glm::vec3(-2.5f, 0.5f, -2.5f), glm::quat());

	headhinge = new btHingeConstraint(*box1->rigidBody, *head->rigidBody, btVector3(0, 2.5f, 0), btVector3(0, -1.1f, 0), btVector3(0, 1, 0), btVector3(0, 1, 0), false);
	//headhinge->setLimit(-glm::radians(20.0f), glm::radians(20.0f));
	dynamicsWorld->addConstraint(headhinge);


	frhinge = new btHingeConstraint(*box1->rigidBody, *frontright->rigidBody, btVector3(2.5f, -2.5f, -2.5f), btVector3(0, 2.6f, 0), btVector3(1, 0.0f, 0), btVector3(1, 0, 0), true);
	frhinge->setLimit(glm::radians(45.0f), glm::radians(90.0f));
	dynamicsWorld->addConstraint(frhinge);


	blhinge = new btHingeConstraint(*box1->rigidBody, *backleft->rigidBody, btVector3(-2.5f, -2.5f, 2.5f), btVector3(0, 2.6f, 0), btVector3(1, 0.0f, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(blhinge);
	blhinge->setLimit(-glm::radians(90.0f), -glm::radians(45.0f));


	brhinge = new btHingeConstraint(*box1->rigidBody, *backright->rigidBody, btVector3(-2.5f, -2.6f, -2.5f), btVector3(0, 2.6f, 0), btVector3(1, 0.0f, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(brhinge);
	brhinge->setLimit(glm::radians(45.0f), glm::radians(90.0f));


	flhinge = new btHingeConstraint(*box1->rigidBody, *frontleft->rigidBody, btVector3(2.5f, -2.6f, 2.5f), btVector3(0, 2.6f, 0), btVector3(1, 0.0f, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(flhinge);
	flhinge->setLimit(-glm::radians(90.0f), -glm::radians(45.0f));

	fhinge1 = new btHingeConstraint(*backleft->rigidBody, *foot1->rigidBody, btVector3(0, -2, 0), btVector3(0, 2, 0), btVector3(1, 0, 0), btVector3(1, 0, 0));
	dynamicsWorld->addConstraint(fhinge1);
	fhinge1->setLimit(-glm::radians(40.0f), glm::radians(60.0f));

	fhinge2 = new btHingeConstraint(*frontleft->rigidBody, *foot2->rigidBody, btVector3(0, -2, 0), btVector3(0, 2, 0), btVector3(1, 0, 0), btVector3(1, 0, 0));
	dynamicsWorld->addConstraint(fhinge2);
	fhinge2->setLimit(-glm::radians(40.0f), glm::radians(60.0f));

	fhinge3 = new btHingeConstraint(*frontright->rigidBody, *foot3->rigidBody, btVector3(0, -2, 0), btVector3(0, 2, 0), btVector3(1, 0, 0), btVector3(1, 0, 0));
	dynamicsWorld->addConstraint(fhinge3);
	fhinge3->setLimit(-glm::radians(40.0f), glm::radians(60.0f));

	fhinge4 = new btHingeConstraint(*backright->rigidBody, *foot4->rigidBody, btVector3(0, -2, 0), btVector3(0, 2, 0), btVector3(1, 0, 0), btVector3(1, 0, 0));
	dynamicsWorld->addConstraint(fhinge4);
	fhinge4->setLimit(-glm::radians(40.0f), glm::radians(60.0f));

	//dynamicsWorld->addConstraint(hinge);

	/*hinge->setLimit(-glm::radians(45.0f), glm::radians(45.0f));*/

	if (!Game::Initialise()) {
		return false;
	}



	return true;
}


void BGE::BoogieBot::Update()
{
	time += Time::deltaTime;

	if (time > 0.5f)
	{
		
		LegPos1();
		

		if (time > 0.75f)
		{
			FootPos1();
		}

		if (time > 1.0f)
		{
			time = 0;
		}
	}
	else
	{
		if (time > 0.25f)
		{
			FootPos2();
		}
		
		LegPos2();
		
	}

	

	
	

	




	Game::Update();
}

void BGE::BoogieBot::Cleanup()
{
	Game::Cleanup();
}


void BGE::BoogieBot::LegPos1()
{
	blhinge->enableAngularMotor(true, -10, force);
	flhinge->enableAngularMotor(true, 10, force);
	brhinge->enableAngularMotor(true, -10, force);
	frhinge->enableAngularMotor(true, 10, force);
}

void BGE::BoogieBot::LegPos2()
{
	blhinge->enableAngularMotor(true, 10, force);
	flhinge->enableAngularMotor(true, -10, force);
	brhinge->enableAngularMotor(true, 10, force);
	frhinge->enableAngularMotor(true, -10, force);
}

void BGE::BoogieBot::FootPos1()
{
	fhinge2->enableAngularMotor(true, -10, force);
	fhinge1->enableAngularMotor(true, 10, force);
	fhinge4->enableAngularMotor(true, -10, force);
	fhinge3->enableAngularMotor(true, 10, force);
}



void BGE::BoogieBot::FootPos2()
{
	fhinge2->enableAngularMotor(true, 10, force);
	fhinge1->enableAngularMotor(true, -10, force);
	fhinge4->enableAngularMotor(true, 10, force);
	fhinge3->enableAngularMotor(true, -10, force);
}