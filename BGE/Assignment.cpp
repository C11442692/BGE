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
	
	shared_ptr<PhysicsController> SpiderBody = physicsFactory->CreateCapsule(2, 4.5f, glm::vec3(0, 15, 0), glm::quat());
	

	shared_ptr<PhysicsController> leg1 = physicsFactory->CreateCapsule(0.6f, 1.5f, glm::vec3(15, 15, 0), glm::quat());
	//shared_ptr<PhysicsController> leg2 = physicsFactory->CreateCapsule(0.6f, 1.5f, glm::vec3(10, 15, 0), glm::quat());

	btHingeConstraint * hinge = new btHingeConstraint(*SpiderBody->rigidBody, *leg1->rigidBody, btVector3(0, 0, 4.5f), btVector3(0, 0, -4.5f), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	hinge->setLimit(glm::radians(90.0f), glm::radians(90.0f));
	dynamicsWorld->addConstraint(hinge);

	if (!Game::Initialise()) {
		return false;
	}

	

	return true;
}
	

void BGE::Assignment::Update()
{
	
	time += Time::deltaTime;
	
	

	

	Game::Update();
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}
