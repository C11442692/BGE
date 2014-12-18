#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class Legs :
		public Game
	{
	private:

	public:
		Legs(void);
		~Legs(void);
		bool Initialise();
		void Update();
		void Cleanup();
		int size;
		int height;
		int width;
		btHingeConstraint * frhinge;
		btHingeConstraint * flhinge;
		btHingeConstraint * brhinge;
		btHingeConstraint * blhinge;
		btHingeConstraint * headhinge;

		btHingeConstraint * fhinge1;
		btHingeConstraint * fhinge2;
		btHingeConstraint * fhinge3;
		btHingeConstraint * fhinge4;

		float time;
	};
}
