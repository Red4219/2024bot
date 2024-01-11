package frc.robot.Tools;

public class AutonomousDetail {
	public String name = "";
	public double startXblue = 0.0;
	public double startYblue = 0.0;
	public double startXred = 0.0;
	public double startYred = 0.0;
	public double startXTolerance = .5;
	public double startYTolerance = .5;

	public AutonomousDetail(
		String name,
		double startXblue,
		double startYblue,
		double startXred,
		double startYred,
		double startXTolerance,
		double startYTolerance
	) {
		this.name = name;
		this.startXblue = startXblue;
		this.startYblue = startYblue;
		this.startXred = startXred;
		this.startYred = startYred;
		this.startXTolerance = startXTolerance;
		this.startYTolerance = startXTolerance;
	}
}
