within AixLib.Fluid.HydraulicModules.Example;
model test_Injection "Test for injection circuit"
  extends Modelica.Icons.Example;

  Injection injection(redeclare package Medium = Medium, pump(redeclare
        AixLib.Fluid.Movers.Data.Pumps.Wilo.CronolineIL80slash220dash4slash4
        per))                                            "hydronic module 1"
    annotation (Placement(transformation(
        extent={{-28,-28},{28,28}},
        rotation=90,
        origin={16,16})));
  replaceable package Medium =
      Modelica.Media.Water.ConstantPropertyLiquidWater
    annotation (__Dymola_choicesAllMatching=true);
  Modelica.Fluid.Sources.Boundary_pT boundary(
    nPorts=1,
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    p=150000,
    T=323.15) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={0,-38})));
  Modelica.Fluid.Sources.Boundary_pT boundary1(
    nPorts=1,
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    T=323.15) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={32,-38})));

  AixLib.Fluid.FixedResistances.PressureDrop hydRes(
    m_flow(start=hydRes.m_flow_nominal),
    dp(start=hydRes.dp_nominal),
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m_flow_nominal=2*996/3600,
    dp_nominal=20000)
    "hydraulic resitance in distribution cirquit (shortcut pipe)" annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={16,64})));
  Modelica.Blocks.Sources.Ramp valveOpening(              duration=500,
      startTime=180)
    annotation (Placement(transformation(extent={{-98,6},{-78,26}})));
  Modelica.Blocks.Sources.Constant RPM(k=2000)
    annotation (Placement(transformation(extent={{-98,44},{-78,64}})));
  HydraulicBus                               hydraulicBus
    annotation (Placement(transformation(extent={{-52,6},{-32,26}})));
equation

  connect(boundary.ports[1], injection.port_fwrdIn) annotation (Line(points={{
          1.77636e-015,-28},{1.77636e-015,-12},{-0.8,-12}},
                                       color={0,127,255}));
  connect(boundary1.ports[1], injection.port_rtrnOut) annotation (Line(points={{32,-28},
          {32,-12},{32.8,-12}},             color={0,127,255}));
  connect(injection.port_fwrdOut, hydRes.port_a)
    annotation (Line(points={{-0.8,44},{-0.8,64},{6,64}},
                                                    color={0,127,255}));
  connect(injection.port_rtrnIn, hydRes.port_b) annotation (Line(points={{32.8,44},
          {30,44},{30,64},{26,64}},         color={0,127,255}));
  connect(valveOpening.y,hydraulicBus. valveSet) annotation (Line(points={{-77,
          16},{-58,16},{-58,16.05},{-41.95,16.05}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(RPM.y, hydraulicBus.rpm_Input) annotation (Line(points={{-77,54},
          {-41.95,54},{-41.95,16.05}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(hydraulicBus, injection.hydraulicBus) annotation (Line(
      points={{-42,16},{-11.72,16}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  annotation (
    Icon(graphics,
         coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=800),
    __Dymola_Commands,
    Documentation(revisions="<html>
<ul>
<li>October 25,2017, by Alexander K&uuml;mpel:<br/>Transfered to AixLib from ZUGABE.</li>
<li>March 7, 2017, by Peter Matthes:<br/>Renamed model instances after renaming of modules.</li>
<li>February 9, 2017, by Peter Matthes:<br/>implemented</li>
</ul>
</html>", info="<html>
<p>Model that demonstrates the use of the injection circuit. The mass flow rate is nearly constant in both the primary and secondary circuit. The injected mass flow increases with the valve opening. </p>
</html>"));
end test_Injection;
