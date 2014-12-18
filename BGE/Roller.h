#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class Roller :
		public Game
	{
	private:

	public:
		Roller(void);
		~Roller(void);
		bool Initialise();
		void Update();
		void Cleanup();
		btHingeConstraint * hinge1;
		btHingeConstraint * hinge2;
		btHingeConstraint * hinge3;
		btHingeConstraint * ballhinge1;
		btHingeConstraint * ballhinge2;
	};
}
