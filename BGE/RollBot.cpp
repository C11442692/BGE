#include "RollBot.h"
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

RollBot::RollBot(void)
{
}

RollBot::~RollBot(void)
{
}


bool RollBot::Initialise()
{
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	setGravity(glm::vec3(0, -9, 0));



	time = 0;
	force = 30;



	shared_ptr<PhysicsController> box1 = physicsFactory->CreateBox(5, 5, 5, glm::vec3(0, 9, 0), glm::quat());
	shared_ptr<PhysicsController> head = physicsFactory->CreateBox(2, 2, 2, glm::vec3(0, 12.7, 0), glm::quat());
	shared_ptr<PhysicsController> frontright = physicsFactory->CreateBox(1.5f, 3.8f, 1.5f, glm::vec3(2.5f, 7.5f, -2.5f), glm::quat());
	shared_ptr<PhysicsController> backleft = physicsFactory->CreateBox(1.5f, 3.8f, 1.5f, glm::vec3(-2.5f, 7.5f, 2.5f), glm::quat());
	shared_ptr<PhysicsController> backright = physicsFactory->CreateBox(1.5f, 3.8f, 1.5f, glm::vec3(-2.5f, 7.5f, -2.5f), glm::quat());
	shared_ptr<PhysicsController> frontleft = physicsFactory->CreateBox(1.5f, 3.8f, 1.5f, glm::vec3(2.5f, 7.5f, 2.5f), glm::quat());
	shared_ptr<PhysicsController> side = physicsFactory->CreateBox(4, 1, 1, glm::vec3(3.6, 10, 0), glm::quat());
	shared_ptr<PhysicsController> side2 = physicsFactory->CreateBox(4, 1, 1, glm::vec3(-8.6, 10, 0), glm::quat());
	shared_ptr<PhysicsController> hangside = physicsFactory->CreateBox(1, 2, 1, glm::vec3(2.5f, 6.5f, 2.5f), glm::quat());
	shared_ptr<PhysicsController> hangside2 = physicsFactory->CreateBox(1, 2, 1, glm::vec3(-7.5f, 6.5f, 2.5f), glm::quat());

	shared_ptr<PhysicsController> ball1 = physicsFactory->CreateSphere(1, glm::vec3(0, 0, 0), glm::quat());
	shared_ptr<PhysicsController> ball2 = physicsFactory->CreateSphere(1, glm::vec3(2, 0, 0), glm::quat());
	shared_ptr<PhysicsController> ball3 = physicsFactory->CreateSphere(1, glm::vec3(4, 0, 0), glm::quat());
	shared_ptr<PhysicsController> ball4 = physicsFactory->CreateSphere(1, glm::vec3(6, 0, 0), glm::quat());


	headhinge = new btHingeConstraint(*box1->rigidBody, *head->rigidBody, btVector3(0, 2.5f, 0), btVector3(0, -1.1f, 0), btVector3(0, 1, 0), btVector3(0, 1, 0), false);

	dynamicsWorld->addConstraint(headhinge);

	sidehinge = new btHingeConstraint(*box1->rigidBody, *side->rigidBody, btVector3(2.6f, 0, 0), btVector3(-3.6f, 0, 1), btVector3(1, 0, 0), btVector3(1, 0, 0), false);
	dynamicsWorld->addConstraint(sidehinge);

	sidehinge2 = new btHingeConstraint(*box1->rigidBody, *side2->rigidBody, btVector3(-2.6f, 0, 0), btVector3(3.6f, 0, 1), btVector3(1, 0, 0), btVector3(1, 0, 0), false);
	dynamicsWorld->addConstraint(sidehinge2);

	offsidehinge = new btHingeConstraint(*side->rigidBody, *hangside->rigidBody, btVector3(0, -1.1f, 0), btVector3(0, 4.2f, 0), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(offsidehinge);

	offsidehinge2 = new btHingeConstraint(*side2->rigidBody, *hangside2->rigidBody, btVector3(0, -1.1f, 0), btVector3(0, 4.2f, 0), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(offsidehinge2);

	frhinge = new btHingeConstraint(*box1->rigidBody, *frontright->rigidBody, btVector3(2.5f, -2.5f, -2.5f), btVector3(0, 2.6f, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	frhinge->setLimit(-glm::radians(15.0f), glm::radians(15.0f));
	dynamicsWorld->addConstraint(frhinge);


	blhinge = new btHingeConstraint(*box1->rigidBody, *backleft->rigidBody, btVector3(-2.5f, -2.5f, 2.5f), btVector3(0, 2.6f, 0), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(blhinge);
	blhinge->setLimit(-glm::radians(0.0f), glm::radians(0.0f));


	brhinge = new btHingeConstraint(*box1->rigidBody, *backright->rigidBody, btVector3(-2.5f, -2.6f, -2.5f), btVector3(0, 2.6f, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(brhinge);
	brhinge->setLimit(-glm::radians(15.0f), glm::radians(15.0f));


	flhinge = new btHingeConstraint(*box1->rigidBody, *frontleft->rigidBody, btVector3(2.5f, -2.6f, 2.5f), btVector3(0, 2.6, 0), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(flhinge);
	flhinge->setLimit(-glm::radians(0.0f), glm::radians(0.0f));

	ballhinge1 = new btHingeConstraint(*frontright->rigidBody, *ball1->rigidBody, btVector3(0, -3.5f, -0), btVector3(0, 0, 0), btVector3(1, 0.0f, 0), btVector3(1, -0, 0), true);
	
	dynamicsWorld->addConstraint(ballhinge1);

	ballhinge2 = new btHingeConstraint(*frontleft->rigidBody, *ball2->rigidBody, btVector3(0, -3.5f, -0), btVector3(0, 0, 0), btVector3(1, 0.0f, 0), btVector3(1, -0, 0), true);
	
	dynamicsWorld->addConstraint(ballhinge2);

	ballhinge3 = new btHingeConstraint(*backleft->rigidBody, *ball3->rigidBody, btVector3(0, -3.5f, -0), btVector3(0, 0, 0), btVector3(1, 0.0f, 0), btVector3(1, -0, 0), true);
	
	dynamicsWorld->addConstraint(ballhinge3);

	ballhinge4 = new btHingeConstraint(*backright->rigidBody, *ball4->rigidBody, btVector3(0, -3.5f, -0), btVector3(0, 0, 0), btVector3(1, 0.0f, 0), btVector3(1, -0, 0), true);
	
	dynamicsWorld->addConstraint(ballhinge4);


	//dynamicsWorld->addConstraint(hinge);

	/*hinge->setLimit(-glm::radians(45.0f), glm::radians(45.0f));*/

	if (!Game::Initialise()) {
		return false;
	}



	return true;
}


void BGE::RollBot::Update()
{



	//sidehinge->enableAngularMotor(true, 2, 40);
	//sidehinge2->enableAngularMotor(true, 2, 40);

	

	flhinge->enableAngularMotor(true, -10, 60);
	brhinge->enableAngularMotor(true, 10, 60);

	frhinge->enableAngularMotor(true, 10, 60);

	

	if (keyState[SDL_SCANCODE_UP])
	{
		ballhinge1->enableAngularMotor(true, 10, 10);
		ballhinge4->enableAngularMotor(true, 10, 10);
		ballhinge2->enableAngularMotor(true, 10, 10);
	}
	else if (keyState[SDL_SCANCODE_DOWN])
	{
		ballhinge1->enableAngularMotor(true, -10, 10);
		ballhinge4->enableAngularMotor(true, -10, 10);
		ballhinge2->enableAngularMotor(true, -10, 10);
	}
	else
	{
		ballhinge1->enableAngularMotor(false, 10, 10);
		ballhinge4->enableAngularMotor(false, 10, 10);
		ballhinge2->enableAngularMotor(false, 10, 10);
	}


	if (keyState[SDL_SCANCODE_LEFT])
	{
		blhinge->setLimit(-glm::radians(15.0f), glm::radians(15.0f));
		flhinge->setLimit(-glm::radians(15.0f), glm::radians(15.0f));

		blhinge->enableAngularMotor(true, 10, 60);
		flhinge->enableAngularMotor(true, 10, 60);

		
	}
	else if (keyState[SDL_SCANCODE_RIGHT])
	{
		blhinge->setLimit(-glm::radians(15.0f), glm::radians(15.0f));
		flhinge->setLimit(-glm::radians(15.0f), glm::radians(15.0f));

		blhinge->enableAngularMotor(true, -10, 60);
		flhinge->enableAngularMotor(true, -10, 60);
	}
	else
	{
		blhinge->setLimit(-glm::radians(0.0f), glm::radians(0.0f));
		flhinge->setLimit(-glm::radians(0.0f), glm::radians(0.0f));

		blhinge->enableAngularMotor(true, -10, 60);
		ballhinge3->enableAngularMotor(true, 10, 10);
	}


	Game::Update();
}

void BGE::RollBot::Cleanup()
{
	Game::Cleanup();
}
