package application;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;

import java.util.concurrent.TimeUnit;


public class tester extends RoboticsAPIApplication {

    private LBR lbr;
    private Controller lbrController;
    private IApplicationUI application_ui;

    private double[] start_position = new double[]{-0.2, 0.6, 0.23, -1.33, -0.12, 1.19, 0};

    private JointImpedanceControlMode jointImp;
    private CartesianImpedanceControlMode catImp;


    public tester(RoboticsAPIContext context) {
        super(context);
    }

    @Override
    public void initialize() {
        this.lbrController = (Controller) getContext().getControllers().toArray()[0];
        this.lbr = (LBR) lbrController.getDevices().toArray()[0];
        this.application_ui = (IApplicationUI) getApplicationUI();
    }

    @Override
    public void run() {

        System.out.println("Start");

        this.lbr.move(ptp(start_position).setJointVelocityRel(0.25));
        Frame start = this.lbr.getCurrentCartesianPosition(this.lbr.getFlange());
        System.out.println(start);

        Frame f = this.lbr.getCurrentCartesianPosition(this.lbr.getFlange());

        double x = 620;
        double y = 0;
        double z = 200;

        f.setX(x).setY(y).setZ(z).setAlphaRad(-Math.PI).setBetaRad(0).setGammaRad(Math.PI);

        this.lbr.move(ptp(f).setJointVelocityRel(0.25));

        System.out.println(this.lbr.getCurrentCartesianPosition(this.lbr.getFlange()));

        //this.lbr.move(ptp(this.lbr.getHomePosition()).setJointVelocityRel(0.5));
        //this.lbr.move(linRel(50,0,0));
        //this.lbr.move(linRel(0,0,20));
        //this.lbr.move(linRel(Transformation.ofDeg(0,0,200,0,0,0)).setJointVelocityRel(0.5));

//		System.out.println(this.lbr.getCurrentCartesianPosition(this.lbr.getFlange()));
//		System.out.println("Impedance!");

        //this.jointImp = new JointImpedanceControlMode(10,10,10,10,10,0,10);
        //this.jointImp.setStiffnessForAllJoints(50);

//		this.catImp = new CartesianImpedanceControlMode();
//		this.catImp.parametrize(CartDOF.Z).setStiffness(100);
//		this.lbr.move(linRel(0,0,300).setJointVelocityRel(0.1));

        ForceCondition force_condition = ForceCondition
                .createSpatialForceCondition(this.lbr.getFlange(), 20);

        ForceCondition force_z = ForceCondition
                .createNormalForceCondition(this.lbr.getFlange(), CoordinateAxis.Z, 4);
        this.lbr.move(linRel(0, 0, 300).breakWhen(force_z).setJointVelocityRel(0.05));

        this.lbr.move(linRel(-100, 0, -200).setJointVelocityRel(0.25));
        boolean result = getObserverManager().waitFor(force_condition, 20, TimeUnit.SECONDS);

		if (result) {
			this.lbr.move(lin(start).setJointVelocityRel(0.25));
		}

        System.out.println("Finish");

    }


    public static void main(String[] args) {
        RoboticsAPIContext.useGracefulInitialization(true);
        RoboticsAPIContext context = RoboticsAPIContext
                .createFromResource(tester.class, "RoboticsAPI.config.xml");
        tester app = new tester(context);
        app.runApplication();
    }

}
