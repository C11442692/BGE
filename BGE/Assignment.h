#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class Assignment :
		public Game
	{
	private:

	public:
		Assignment(void);
		~Assignment(void);
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
		btScalar force;
		float time;
	};
}
