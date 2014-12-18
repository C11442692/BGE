#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class RollBot :
		public Game
	{
	private:

	public:
		RollBot(void);
		~RollBot(void);
		bool Initialise();
		void Update();
		void Cleanup();
		void CreateWall(int, int, int);
		int size;
		int height;
		int width;
		btHingeConstraint * frhinge;
		btHingeConstraint * flhinge;
		btHingeConstraint * brhinge;
		btHingeConstraint * blhinge;
		btHingeConstraint * sidehinge;
		btHingeConstraint * sidehinge2;
		btHingeConstraint * offsidehinge;
		btHingeConstraint * offsidehinge2;
		btHingeConstraint * headhinge;
		btHingeConstraint * ballhinge1;
		btHingeConstraint * ballhinge2;
		btHingeConstraint * ballhinge3;
		btHingeConstraint * ballhinge4;
		btScalar force;
		float time;
	};
}
