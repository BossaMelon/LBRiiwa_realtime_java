package application;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;

import java.util.concurrent.TimeUnit;


public class Tester extends RoboticsAPIApplication {

    private LBR lbr;
    private Controller lbrController;
    private IApplicationUI application_ui;

    private double[] start_position = new double[]{-0.2, 0.6, 0.23, -1.33, -0.12, 1.19, 0};
    private double[] zero_postion = new double[]{0, 0, 0, 0, 0, 0, 0};

    private JointImpedanceControlMode jointImp;
    private CartesianImpedanceControlMode catImp;


    public Tester(RoboticsAPIContext context) {
        super(context);
    }

    @Override
    public void initialize() {
        this.lbrController = (Controller) getContext().getControllers().toArray()[0];
        this.lbr = (LBR) lbrController.getDevices().toArray()[0];
        this.application_ui = (IApplicationUI) getApplicationUI();
    }

    @Override
    // only need to add native lbr iiwa function inside run()
    public void run() {

        System.out.println("Start");

        this.lbr.move(ptp(zero_postion).setJointVelocityRel(0.25));

        this.lbr.move(ptp(start_position).setJointVelocityRel(0.25));

        Frame current = this.lbr.getCurrentCartesianPosition(this.lbr.getFlange());
        System.out.println(current);

        this.lbr.move(ptp(zero_postion).setJointVelocityRel(0.25));

        System.out.println("Finish");
        
    }


    public static void main(String[] args) {
        RoboticsAPIContext.useGracefulInitialization(true);
        RoboticsAPIContext context = RoboticsAPIContext
                .createFromResource(Tester.class, "RoboticsAPI.config.xml");
        Tester app = new Tester(context);
        app.runApplication();
    }

}
