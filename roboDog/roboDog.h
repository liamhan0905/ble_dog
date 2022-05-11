class RoboDog
{
  private:  
    int hipHomePos [4];
    int hipMinPos [4];
    int hipMaxPos [4];
    int shoulderHomePos [4];
    int shoulderMinPos [4];
    int shoulderMaxPos [4];
    int elbowHomePos [4];
    int elbowMinPos [4];
    int elbowMaxPos [4];

  public:
    RoboDog();
    stand();
    crouch();
    home();
}
