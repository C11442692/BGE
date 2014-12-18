#include "Assignment.h"
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

Assignment::Assignment(void)
{
}

Assignment::~Assignment(void)
{
}

std::shared_ptr<GameComponent> station2;

bool Assignment::Initialise()
{
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	setGravity(glm::vec3(0, -9, 0));

	

	time = 0;
	force = 30;


	
	shared_ptr<PhysicsController> box1 = physicsFactory->CreateBox(5, 5, 5 , glm::vec3(0, 7, 0), glm::quat());
	shared_ptr<PhysicsController> head = physicsFactory->CreateBox(2, 2, 2, glm::vec3(0, 12.7, 0), glm::quat());
	shared_ptr<PhysicsController> frontright = physicsFactory->CreateBox(1.5f, 3.8f, 1.5f, glm::vec3(2.5f, 7.5f, -2.5f), glm::quat());
	shared_ptr<PhysicsController> backleft = physicsFactory->CreateBox(1.5f, 3.8f, 1.5f, glm::vec3(-2.5f, 7.5f, 2.5f), glm::quat());
	shared_ptr<PhysicsController> backright = physicsFactory->CreateBox(1.5f, 3.8f, 1.5f, glm::vec3(-2.5f, 7.5f, -2.5f), glm::quat());
	shared_ptr<PhysicsController> frontleft = physicsFactory->CreateBox(1.5f, 3.8f, 1.5f, glm::vec3(2.5f, 7.5f, 2.5f), glm::quat());
	shared_ptr<PhysicsController> side = physicsFactory->CreateBox(4, 1, 1, glm::vec3(3.6, 10, 0), glm::quat());
	shared_ptr<PhysicsController> side2 = physicsFactory->CreateBox(4, 1, 1, glm::vec3(-8.6, 10, 0), glm::quat());
	shared_ptr<PhysicsController> hangside = physicsFactory->CreateBox(1, 4, 1, glm::vec3(2.5f, 6.5f, 2.5f), glm::quat());
	shared_ptr<PhysicsController> hangside2 = physicsFactory->CreateBox(1, 4, 1, glm::vec3(-7.5f, 6.5f, 2.5f), glm::quat());

	headhinge = new btHingeConstraint(*box1->rigidBody, *head->rigidBody, btVector3(0, 2.5f, 0), btVector3(0, -1.1f, 0), btVector3(0, 1, 0), btVector3(0, 1, 0), false);
	
	dynamicsWorld->addConstraint(headhinge);

	sidehinge = new btHingeConstraint(*box1->rigidBody, *side->rigidBody, btVector3(2.6f, 0, 0), btVector3(-3.6f, 0, 0), btVector3(1, 0, 0), btVector3(1,0,0), false);
	dynamicsWorld->addConstraint(sidehinge);

	btTransform test;
	btTransform test2;

	

	test.setIdentity();
	test2.setIdentity();

	test2.setOrigin(btVector3(13, 0, 0));
	//test2.setRotation((GLToBtQuat(glm::angleAxis(180.0f, glm::vec3(1, 0, 0)))));

	connect = new btFixedConstraint(*side->rigidBody, *side2->rigidBody, test , test2);
	dynamicsWorld->addConstraint(connect);

	offsidehinge = new btHingeConstraint(*side->rigidBody, *hangside->rigidBody, btVector3(0, -1.1f, 0), btVector3(0, 4.2f, 0), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(offsidehinge);

	offsidehinge2 = new btHingeConstraint(*side2->rigidBody, *hangside2->rigidBody, btVector3(0, -1.1f, 0), btVector3(0, 4.2f, 0), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(offsidehinge2);

	frhinge = new btHingeConstraint(*box1->rigidBody, *frontright->rigidBody, btVector3(2.5f, -2.5f, -2.5f), btVector3(0, 2.6f, 0), btVector3(1, 0.0f, 0), btVector3(1, -0.5f, 0.5f), true);
	frhinge->setLimit(-glm::radians(15.0f), glm::radians(15.0f));
	dynamicsWorld->addConstraint(frhinge);
	
	
	blhinge = new btHingeConstraint(*box1->rigidBody, *backleft->rigidBody, btVector3(-2.5f, -2.5f, 2.5f), btVector3(0, 2.6f, 0), btVector3(1, 0.0f, 0), btVector3(1, 0.5f, 0.5f), true);
	dynamicsWorld->addConstraint(blhinge);
	blhinge->setLimit(-glm::radians(15.0f), glm::radians(15.0f));
	
	
	brhinge = new btHingeConstraint(*box1->rigidBody, *backright->rigidBody, btVector3(-2.5f, -2.6f, -2.5f ), btVector3(0, 2.6f, 0), btVector3(1, 0.0f, 0), btVector3(1, 0.5f, 0.5f), true);
	dynamicsWorld->addConstraint(brhinge);
	brhinge->setLimit(-glm::radians(15.0f), glm::radians(15.0f));

	
	flhinge = new btHingeConstraint(*box1->rigidBody, *frontleft->rigidBody, btVector3(2.5f, -2.6f, 2.5f), btVector3(0, 2.6f, 0), btVector3(1, 0.0f, 0), btVector3(1, -0.5f, 0.5f), true);
	dynamicsWorld->addConstraint(flhinge);
	flhinge->setLimit(-glm::radians(15.0f), glm::radians(15.0f));


	//dynamicsWorld->addConstraint(hinge);

	/*hinge->setLimit(-glm::radians(45.0f), glm::radians(45.0f));*/

	if (!Game::Initialise()) {
		return false;
	}

	

	return true;
}
	

void BGE::Assignment::Update()
{
	
	time += Time::deltaTime;
	
	sidehinge->enableAngularMotor(true, 3, 50);
	//sidehinge2->enableAngularMotor(true, 2, 10000);


	flhinge->enableAngularMotor(true, -10, 60);
	brhinge->enableAngularMotor(true, 10, 60);

	frhinge->enableAngularMotor(true, 10, 60);

	blhinge->enableAngularMotor(true, -10, 60);

	

	Game::Update();
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}
