#include "Roller.h"
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

Roller::Roller(void)
{
}

Roller::~Roller(void)
{
}


bool Roller::Initialise()
{
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	setGravity(glm::vec3(0, -9, 0));





	shared_ptr<PhysicsController> part1 = physicsFactory->CreateBox(1, 5, 1, glm::vec3(0, 5, 0), glm::quat());
	shared_ptr<PhysicsController> part2 = physicsFactory->CreateBox(1, 5, 1, glm::vec3(2, 5, 0), glm::quat());
	shared_ptr<PhysicsController> part3 = physicsFactory->CreateBox(1, 5, 1, glm::vec3(4, 5, 0), glm::quat());
	shared_ptr<PhysicsController> part4 = physicsFactory->CreateBox(1, 5, 1, glm::vec3(6, 5, 0), glm::quat());

	shared_ptr<PhysicsController> ball1 = physicsFactory->CreateSphere(0.5f, glm::vec3(0, 0, 0), glm::quat());
	shared_ptr<PhysicsController> ball2 = physicsFactory->CreateSphere(0.5f, glm::vec3(6, 0, 0), glm::quat());


	hinge1 = new btHingeConstraint(*part1->rigidBody, *part2->rigidBody, btVector3(1, 0, 0), btVector3(-1, 0, 0), btVector3(1, 0.0f, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(hinge1);

	hinge2 = new btHingeConstraint(*part2->rigidBody, *part3->rigidBody, btVector3(1, 0, 0), btVector3(-1, 0, 0), btVector3(1, 0.0f, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(hinge2);

	hinge3 = new btHingeConstraint(*part3->rigidBody, *part4->rigidBody, btVector3(1, 0, 0), btVector3(-1, 0, 0), btVector3(1, 0.0f, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(hinge3);

	ballhinge1 = new btHingeConstraint(*part1->rigidBody, *ball1->rigidBody, btVector3(0, -2.5f, 0), btVector3(0, 1, 0), btVector3(1, 0.0f, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(ballhinge1);

	ballhinge2 = new btHingeConstraint(*part4->rigidBody, *ball2->rigidBody, btVector3(0, -2.5f, 0), btVector3(0, 1, 0), btVector3(1, 0.0f, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(ballhinge2);

	ballhinge1->setLimit(-glm::radians(0.0f), glm::radians(0.0f));
	ballhinge2->setLimit(-glm::radians(0.0f), glm::radians(0.0f));

	//dynamicsWorld->addConstraint(hinge);

	/*hinge->setLimit(-glm::radians(45.0f), glm::radians(45.0f));*/

	if (!Game::Initialise()) {
		return false;
	}



	return true;
}


void BGE::Roller::Update()
{

	hinge1->enableAngularMotor(true, -5, 20);
	hinge3->enableAngularMotor(true, 5, 20);



	Game::Update();
}

void BGE::Roller::Cleanup()
{
	Game::Cleanup();
}
