///////////////////////////
// Electric hobby engine //
///////////////////////////
class ElectricEng : public Propulsion
{
  public:
    ////////////////
    // Variables //
    ////////////////
    double omegaMin, omegaMax;
    double propDiam, engInertia, rho;
    double Kv, Rm, I0;
    double mass;
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
    ElectricEng(Model * parent,int id);
    ~ElectricEng();

    void updateRadPS();
    void calcThrust();
    void calcTorque();
    void calcOmega();
};