#include "project.h"

extern int j;

void Tracking::stateMachine()
{
	switch(_currentState)
	{
		case StateInitState:
			{
				if(!ardrone.onGround())
					setState(StateInitHover);
				//printf("moving to init hover \ n");
				break;
			}
		case StateInitHover:
			{
				if(j < (_currentState & 0x30 ? 50 : 500000))
				{
					if(j % 1000 == 0)
						printf(".");

					j++;
					ardrone.move2D(0.0f, 0.0f, 0.0f);
				}
				else
				{
					printf("\n");
					setState(StateSetAltitude);
				}

				break;
			}
		case StateSetAltitude:
			{
				if (ardrone.getAltitude() >= 1.7f)
				{
					averager.clear();
					setState(StateFCSearching);
					j = 50;
				}
				else
				{
					ardrone.move3D(0.0f, 0.0f, 0.9f, 0.0f);
				}
				
				break;
			}
		case StateFCSearching:
			{
				if(target.isFound)
				{
					if(target.distance < 8.5f)
						setState(StateFCFollowing);
					else if(target.distance < 15.0f)
						setState(StateFCMoving);
				}

				float speed = getVerticalSpeed();

				if(j < 50)
				{
					j++;
					// ardrone.move3D(0.0f, 0.0f, speed, 0.0f);
					ardrone.move3D(-0.06f, 0.0f, speed, 0.0f);
				}
				else
				{
					ardrone.move3D(0.0f, 0.0f, speed, -0.65f);
				}

				break;
			}
		case StateFCMoving:
			{

				if(!target.isFound)
				{
					averager.clear();
					setState(StateFCSearching);
				}
				else if(target.distance < 8.5f)
				{
					ardrone.move3D(0.0f, 0.0f, getVerticalSpeed(), 0.0f);
					j = 45;
					setState(StateFCBreaking);
				}
				else
				{
					doMovement();
				}

				break;
			}
		case StateFCFollowing:
			{
				j++;
				if(!target.isFound)
				{
					averager.clear();
					setState(StateFCSearching);
				}
				else if(target.distance > 10.5f)
				{
					ardrone.move3D(0.0f, 0.0f, getVerticalSpeed(), 0.0f);
					setState(StateFCMoving);
				}
				else
					followTarget(j);

				break;
			}
		case StateFCBreaking:
			{
					
					//ardrone.move3D(-0.5f, 0.0f, getVerticalSpeed(), 0.0f);
					for (int i = 0; i < 10000; i++)
					{
						if (i%100)
						{
							ardrone.move3D(-0.5f, 0.0f, getVerticalSpeed(), 0.0f);
						}
					}
					setState(StateFCFollowing);
				break;
			}
/*
		case StateFCLostTarget: // Not Using
			{
				if(target.distance <= 5)
					CurrentState = StateFCFollowing;
				else if(target.distance <= 20)
					CurrentState = StateFCMoving;
						
				ardrone.move2D(0.0f, 0.0f, 0.0f);
				break;
			}
		case StateFCAiming: // Not Using
			{
				float aim = 0.0f;
				int x = averager.lastAverage.x;
				if(x >= 25 && x < 25)
				{
					aim = (x / 640.0) * 2.5f;
				}
				else
				{
					CurrentState = StateBTForwardHover;
				}

				ardrone.move2D(0.0f, 0.0f, aim);
				break;
			}
		case StateBTForwardHover: // Not Using
			{
				if(target.distance < 20)
				{
					CurrentState = StateBTMoving;
				}
				if(j++ > 100)
				{
					CurrentState = StateBTSearching;
					j = 0;
				}

				//printf("BT Forward Hover ........ Count = %d\n", j);
				ardrone.move2D(0.5f, 0.0f, 0.0f);
				break;
			}
		case StateBTMoving: // Not Using
			{
				if(target.distance > 20)
				{
					CurrentState = StateBTSearching;
				}
				else if(KEY_PUSH('D') || KEY_PUSH('d'))
				{
					CurrentState = StateLanding;
				}

				followTarget();
				break;
			}
		case StateBTSearching: // Not Using
			{
				if(target.distance <= 20)
				{
					CurrentState = StateBTMoving;
				}

				//printf("BT Searching ............ \n");
				ardrone.move2D(0.0f, 0.0f, 0.0f);
				break;
			}
		case StateLanding: // Not Using
			{
				if(j++ > 2000000)
				{
					CurrentState = StateInitState;
					ardrone.landing();
					j = 0;
				}

				ardrone.move2D(0.0f, 0.0f, 0.0f);
				break;
			}
*/
	}
};