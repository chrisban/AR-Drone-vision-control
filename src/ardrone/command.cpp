#include "ardrone.h"

// --------------------------------------------------------------------------
// ARDrone::initCommand()
// Initialize AT Command.
// Return value SUCCESS: 1  FAILED: 0
// --------------------------------------------------------------------------
int ARDrone::initCommand(void)
{
    // Open the socket
    if (!sockCommand.open(ip, ARDRONE_COMMAND_PORT)) {
        printf("ERROR: UDPSocket::open(port=%d) failed. (%s, %d)\n", ARDRONE_COMMAND_PORT, __FILE__, __LINE__);
        return 0;
    }

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::takeoff()
// Take off the AR.Drone.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::takeoff(void)
{//                    AT*REF=12,290718208
    sockCommand.sendf("AT*REF=%d,290718208\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::landing()
// Land the AR.Drone.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::landing(void)
{
    sockCommand.sendf("AT*REF=%d,290717696\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::emergency()
// Emergency stop.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::emergency(void)
{
    sockCommand.sendf("AT*REF=%d,290717952\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::move(X velocity[m/s], Y velocity[m/s], Rotational speed[rad/s])
// Move the AR.Drone in 2D plane.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::move2D(float vx, float vy, float vr)
{
    move3D(vx, vy, 0.0, vr);
}

// --------------------------------------------------------------------------
// ARDrone::move3D(X velocity[m/s], Y velocity[m/s], Z velocity[m/s], Rotational speed[rad/s])
// Move the AR.Drone in 3D space.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::move3D(float vx, float vy, float vz, float vr)
{
	assert(sizeof(int)==sizeof(float));
	const float gain = 0.4f;
  float v[4] = {-vy*gain, -vx*gain, vz*gain, vr*gain};
  int mode = ((fabs(vx) > 0.001f) || (fabs(vy) > 0.001f));
  //sockCommand.sendf("AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq++, *(int*)(&mode), *(int*)(&v[0]), *(int*)(&v[1]), *(int*)(&v[2]), *(int*)(&v[3]));
  sockCommand.sendf("AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq++, mode, *(int*)(&v[0]), *(int*)(&v[1]), *(int*)(&v[2]), *(int*)(&v[3]));
  //sockCommand.sendf("AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq++, mode, -vy*gain, -vx*gain, *(int*)(&v[2]), *(int*)(&v[3]));
	/*
	float gain = 0.4f;
	int mode;

	if(vx == 0 && vy == 0 && vz == 0 && vr == 0)
		mode = 0;
	else
		mode = 1;

  sockCommand.sendf("AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq++, mode, -vx*gain, -vy*gain, vz*gain, -vr*gain);
	//sockCommand.sendf("AT*PCMD_MAG=%d,%d,%d,%d,%d,%d,%d,%d\r", seq++, mode, -vx*gain, -vy*gain, vz*gain, -vr*gain, 0.0f, 0.0f);
	*/
}

int ARDrone::getSequenceNumber()
{
	int s = seq;
	seq++;
	return s;
}

// --------------------------------------------------------------------------
// ARDrone::setLED(LED animation ID, Frequency[Hz], Duration[s])
// Run specified LED animation.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::setLED(int id, float freq, int duration)
{
    sockCommand.sendf("AT*LED=%d,%d,%d,%d\r", seq++, id, *(int*)(&freq), duration);
    //sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
    //sockCommand.sendf("AT*CONFIG=%d,\"leds:leds_anim\",\"%d,%d,%d\"\r", seq++, id, *(int*)(&freq), duration);
    //Sleep(100);
}

// --------------------------------------------------------------------------
// ARDrone::setCamera(Channel)
// Change the camera channel.
// ARDrone1.0 supports 0 or 2.
// ARDrone2.0 supports 0 or 1.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::setCamera(int mode)
{
    // ARDrone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"%d\"\r", seq++, mode);
        Sleep(100);
    }
    // ARDrone 1.0
    else {
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"%d\"\r", seq++, mode);
    }
}

// --------------------------------------------------------------------------
// ARDrone::resetWatchDog()
// Stop hovering.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::resetWatchDog(void)
{
    if (navdata.ardrone_state & ARDRONE_COM_WATCHDOG_MASK) sockCommand.sendf("AT*COMWDG=%d\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::resetEmergency()
// Disable the emergency lock.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::resetEmergency(void)
{
    if (navdata.ardrone_state & ARDRONE_EMERGENCY_MASK) sockCommand.sendf("AT*REF=%d,290717952\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::finalizeCommand()
// Finalize AT command
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::finalizeCommand(void)
{
    // Close the socket
    sockCommand.close();
}