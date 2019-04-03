class Environment
{
    public:
        last_letter_2_msgs::air_data airdata;
        last_letter_2_msgs::link_states states;
        double windDir, windRef, windRefAlt,surfSmooth, kwind;
        double T0; //Temperature at sea level, degrees K
        double P0; //Pressure at sea level, in HG
        double rho; //Density at sea level, kg/m**3
        double windDistU;
        double windDistV[2], windDistW[2];
        double Lu,Lw,sigmau,sigmaw;
        double allowTurbulence, dt, grav0;
        Model * model;
        Environment(Model * model);
        void calculateAirdata();
        void calcWind();
        void calcDens();
        void calcPres();
        void calcTemp();

};

