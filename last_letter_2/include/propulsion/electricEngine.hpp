///////////////////////////
// Electric hobby engine //
///////////////////////////
class electricEngine : public Propulsion
{
  public:
    ////////////////
    // Variables //
    ////////////////
    double omegaMin, omegaMax;
    double propDiam, engInertia, rho;
    double Kv, Rm, I0;
    double maxThrust;
    // Battery specification
    double omega; // motor angular speed in rad/s
    int Cells;          // Number of LiPo cells
    double Rs;          // Battery internal resistance
    double rotationDir; // motor direction of rotation

    // ros::Publisher pub;

    //////////////
    // Members //
    //////////////
    Polynomial *engPowerPoly, *npPoly, *propPowerPoly;

    ////////////////
    // Functions //
    ////////////////
    electricEngine(Model * parent,int id);
    ~electricEngine();

    void calcThrust();
    void calcTorque();
    void calcOmega();
};