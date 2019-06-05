
class NoEngine : public Propulsion
 {
   public:
   NoEngine(Model * parent, int id);
   void calcThrust();
   void calcTorque();
   void calcOmega();
 };