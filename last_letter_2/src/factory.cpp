#include <XmlRpcValue.h>

//build and return an aerodynamics object based on aerodynamicsType of the airfoil
Aerodynamics *Factory::buildAerodynamics(Model *parent, int id)
{
    char paramMsg[50];
    int type;
    sprintf(paramMsg, "airfoil%i/aerodynamicsType", id+1);
    if (!ros::param::getCached(paramMsg, type)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    switch (type)
    {
    case 0:
        return new NoAerodynamics(parent,id+1);
    case 1:
        return new StdLinearAero(parent,id+1);
    case 2:
        return new polyAero(parent,id+1);
    }
}

//build and return a propulsion object based on propulsioType of the motor
Propulsion *Factory::buildPropulsion(Model *parent, int id)
{
    char paramMsg[50];
    int type;
    sprintf(paramMsg, "motor%i/motorType", id+1);
    if (!ros::param::getCached(paramMsg, type)) { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
    switch (type)
    {
    case 0:
        return new NoEngine(parent, id+1);
    case 1:
        return new genericEngine(parent, id+1);
    // case 2: return new PistonEngin(parent, id+1);
    case 3:
        return new ElectricEng(parent, id+1);
    }
}

// // Build a new polynomial, reading from the paramter server
Polynomial *Factory::buildPolynomial(char *baseParam)
{
    int i;
    XmlRpc::XmlRpcValue list;
    char parameter[100];
    sprintf(parameter, "%s/%s", baseParam, "polyType");
    if (!ros::param::getCached(parameter, i)) { ROS_FATAL("Invalid parameters for %s in param server!", parameter); ros::shutdown();}
    switch (i)
    {
    case 0:
    {
        std::cout << "selecting 1D polynomial" << std::endl;
        sprintf(parameter, "%s/%s", baseParam, "polyNo");
        if (!ros::param::getCached(parameter, i)) { ROS_FATAL("Invalid parameters for %s in param server!", parameter); ros::shutdown(); }
        int polyNo = i;
        sprintf(parameter, "%s/%s", baseParam, "coeffs");
        if (!ros::param::getCached(parameter, list)) { ROS_FATAL("Invalid parameters for %s in param server!", parameter); ros::shutdown(); }
        double coeffs[polyNo + 1];
        if (polyNo + 1 != list.size())
        {
            ROS_FATAL("Polynomial order and provided coefficient number do not match");
            ros::shutdown();
        }
        for (i = 0; i <= polyNo; i++)
        {
            ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            coeffs[i] = list[i];
        }
        return new Polynomial1D(polyNo, coeffs);
    }
    case 1:
    {
        std::cout << "selecting 2D polynomial" << std::endl;
        sprintf(parameter, "%s/%s", baseParam, "polyNo");
        if (!ros::param::getCached(parameter, list)) { ROS_FATAL("Invalid parameters for %s in param server!", parameter); ros::shutdown(); }
        ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeInt);
        int polyOrder1 = list[0];
        int polyOrder2 = list[1];
        sprintf(parameter, "%s/%s", baseParam, "coeffs");
        if (!ros::param::getCached(parameter, list)) { ROS_FATAL("Invalid parameters for %s in param server!", parameter); ros::shutdown(); }
        int length = list.size();
        if ((2 * polyOrder2 + 2 * polyOrder1 * polyOrder2 + polyOrder1 - polyOrder1 * polyOrder1 + 2) / 2 != length)
        {
            ROS_FATAL("Polynomial order and provided coefficient number do not match");
            ros::shutdown();
        }
        double coeffs[length];
        for (i = 0; i < list.size(); ++i)
        {
            ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            coeffs[i] = list[i];
        }
        return new Polynomial2D(polyOrder1, polyOrder2, coeffs);
    }
    case 2:
    {
        std::cout << "selecting cubic spline" << std::endl;
        sprintf(parameter, "%s/%s", baseParam, "breaksNo");
        if (!ros::param::getCached(parameter, i)) { ROS_FATAL("Invalid parameters for %s in param server!", parameter); ros::shutdown(); }
        int breaksNo = i;
        sprintf(parameter, "%s/%s", baseParam, "breaks");
        if (!ros::param::getCached(parameter, list)) { ROS_FATAL("Invalid parameters for %s in param server!", parameter); ros::shutdown(); }
        if ((breaksNo + 1) != list.size())
        {
            ROS_FATAL("Spline breaks order and provided breaks number do not match");
            ros::shutdown();
        }
        double breaks[list.size()];
        for (i = 0; i < list.size(); ++i)
        {
            ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            breaks[i] = list[i];
        }
        sprintf(parameter, "%s/%s", baseParam, "coeffs");
        if (!ros::param::getCached(parameter, list)) { ROS_FATAL("Invalid parameters for %s in param server!", parameter); ros::shutdown(); }
        if (breaksNo * 4 != list.size())
        {
            ROS_FATAL("breaks order and provided coeffs number do not match");
            ros::shutdown();
        }
        double coeffs[list.size()];
        for (i = 0; i < list.size(); ++i)
        {
            ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            coeffs[i] = list[i];
        }
        return new Spline3(breaksNo, breaks, coeffs);
    }
    default:
    {
        ROS_FATAL("Error while constructing a polynomial");
        ros::shutdown();
        break;
    }
    }
}
