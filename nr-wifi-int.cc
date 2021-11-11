/* Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; */
/*
 *   Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *   Copyright (c) 2021 Orange Labs
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "simulation-helper.h"
#include <ns3/log.h>
#include <fstream>
#include <iostream>
#include <string>
#include <ns3/nr-u-module.h>
#include <ns3/nr-module.h>
#include <ns3/wifi-module.h>
#include <ns3/internet-module.h>
#include <ns3/flow-monitor-module.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/applications-module.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("CttcNrWifiInterferenceExample");

static const uint32_t PACKET_SIZE = 1000;

static void
ConfigureDefaultValues (bool cellScan = true, double beamSearchAngleStep = 10.0,
                        const std::string &errorModel = "ns3::NrEesmErrorModel",
                        double cat2EDThreshold = -69.0, double cat3and4EDThreshold = -79.0,
                        const std::string & rlcModel = "RlcTmAlways")
{
  Config::SetDefault ("ns3::ThreeGppPropagationLossModel::ShadowingEnabled",
                      BooleanValue (false));

  if (cellScan)
    {
      Config::SetDefault ("ns3::IdealBeamformingHelper::BeamformingMethod", TypeIdValue (CellScanBeamforming::GetTypeId ()));
    }
  else
    {
      Config::SetDefault ("ns3::IdealBeamformingHelper::BeamformingMethod", TypeIdValue (DirectPathBeamforming::GetTypeId ()));
    }
  //Config::SetDefault ("ns3::IdealBeamformingHelper::BeamformingPeriodicity",
  //                    TimeValue (MilliSeconds (1000))); //seldom updated
  Config::SetDefault ("ns3::CellScanBeamforming::BeamSearchAngleStep",
                      DoubleValue (beamSearchAngleStep));

  Config::SetDefault ("ns3::UniformPlanarArray::NumColumns", UintegerValue (2));
  Config::SetDefault ("ns3::UniformPlanarArray::NumRows", UintegerValue (2));

  Config::SetDefault ("ns3::NrGnbPhy::NoiseFigure", DoubleValue (7));
  Config::SetDefault ("ns3::NrUePhy::NoiseFigure", DoubleValue (7));
  Config::SetDefault ("ns3::WifiPhy::RxNoiseFigure", DoubleValue (7));

  Config::SetDefault ("ns3::IsotropicAntennaModel::Gain", DoubleValue (0));
  Config::SetDefault ("ns3::WifiPhy::TxGain", DoubleValue (0));
  Config::SetDefault ("ns3::WifiPhy::RxGain", DoubleValue (0));

  Config::SetDefault ("ns3::NrSpectrumPhy::UnlicensedMode", BooleanValue (true));

  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize",
                      UintegerValue (999999999));
  Config::SetDefault ("ns3::LteRlcTm::MaxTxBufferSize",
                      UintegerValue (999999999));

  Config::SetDefault ("ns3::PointToPointEpcHelper::S1uLinkDelay", TimeValue (MilliSeconds (0)));
  Config::SetDefault ("ns3::NoBackhaulEpcHelper::X2LinkDelay", TimeValue (MilliSeconds (0)));
  Config::SetDefault ("ns3::LteEnbRrc::EpsBearerToRlcMapping",  StringValue (rlcModel));

  Config::SetDefault ("ns3::NrAmc::ErrorModelType", TypeIdValue (TypeId::LookupByName (errorModel)));
  Config::SetDefault ("ns3::NrAmc::AmcModel", EnumValue (NrAmc::ShannonModel));
  Config::SetDefault ("ns3::NrSpectrumPhy::ErrorModelType", TypeIdValue (TypeId::LookupByName (errorModel)));

  /* Global params: no fragmentation, no RTS/CTS, fixed rate for all packets */
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("999999"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));

  Config::SetDefault ("ns3::ApWifiMac::EnableBeaconJitter", BooleanValue (true));

  // Cat3 and Cat 4 ED treshold, Cat2 has its own attribute, maybe in future each could have its own
  Config::SetDefault ("ns3::NrLbtAccessManager::EnergyDetectionThreshold", DoubleValue (cat3and4EDThreshold));
  // Cat2 ED threshold
  Config::SetDefault ("ns3::NrCat2LbtAccessManager::Cat2EDThreshold", DoubleValue (cat2EDThreshold));
}

static void
TimePasses ()
{
  time_t t = time (nullptr);
  struct tm tm = *localtime (&t);

  std::cout << "Simulation Time: " << Simulator::Now ().As (Time::S) << " real time: "
            << tm.tm_hour << "h, " << tm.tm_min << "m, " << tm.tm_sec << "s." << std::endl;
  Simulator::Schedule (MilliSeconds (100), &TimePasses);
}

static bool m_nrIsOccupying = false;
static bool m_wifiIsOccupying = false;
static Time m_nrOccupancy;
static Time m_wifiOccupancy;
static OutputManager *outputManager;

static void
ResetNrOccupancy ()
{
  m_nrIsOccupying = false;
}

static void
ResetWifiOccupancy ()
{
  m_wifiIsOccupying = false;
}

/*
static void
PrintScenario(uint32_t gnbs, uint32_t ues)
{
  uint32_t numGnbRows = gnbs / 2;
  uint32_t numUeRows = ues / 3;
  uint32_t ueC = 0;
  //uint32_t gnbC = 0;

  for(int i =0; i < 10 * (numGnbRows + numUeRows + 2); i++){
    for(int j= 0; j < 17; j ++)
    {
      if (j % 4 == 0 && i % 9 == 0)
      {
        if(ueC < (ues / 2))
        {
          ueC++;
          std::cout << "X";
        } else {
          std::cout << "X";
        }
      } 
    }

    std::cout << std::endl;
  }
}
*/


static void
NrOccupancy (uint32_t nodeId, const Time & time)
{
  outputManager->UidIsTxing (nodeId);
  if (m_wifiIsOccupying)
    {
      outputManager->SimultaneousTxOtherTechnology (nodeId);
      std::cout << "There was a sim tx from other tech wifi" << std::endl;
    }

  if (m_nrIsOccupying)
    {
      outputManager->SimultaneousTxSameTechnology (nodeId);
      std::cout << "There was a sim tx from same tech nr" << std::endl;
    }

  m_nrOccupancy += time;
  m_nrIsOccupying = true;
  Simulator::Schedule (time, &ResetNrOccupancy);
}

static Ptr<Node>
GetClosestAp (Ptr<Node> ueDevice, const NodeContainer &apDevices)
{
  
  NS_ASSERT_MSG (apDevices.GetN () > 0, "empty ap device container");
  Vector uepos = ueDevice->GetObject<MobilityModel> ()->GetPosition ();
  double minDistance = std::numeric_limits<double>::infinity ();
  Ptr<Node> closestApDevice;
  for (auto i = apDevices.Begin (); i != apDevices.End (); ++i)
  {
    Vector enbpos = (*i)->GetObject<MobilityModel> ()->GetPosition ();
    double distance = CalculateDistance (uepos, enbpos);
    if (distance < minDistance)
    {
      minDistance = distance;
      closestApDevice = (*i);
    }
  }
  NS_ASSERT (closestApDevice != nullptr);
  return closestApDevice;
}

/*
static void
SinrInfo (uint32_t nodeId, uint32_t sinV)
{
  std::cout << "This is the node ID " << nodeId << " And this is the SINR value " << sinV << std::endl;

}

static void
MacTxDataFailed (uint32_t nodeId, uint32_t bytes)
{
  std::cout << "This is the node ID " << nodeId << " And this is the bytes " << bytes << std::endl;

}

static void
ChannelReqTime (uint32_t nodeId, Time value)
{
  std:: cout << "This is the node ID " << nodeId << " And this is the channel request time " << value << std::endl;
}

*/
static void
WifiOccupancy (uint32_t nodeId, const Time & time)
{
  outputManager->UidIsTxing (nodeId);
  if (m_nrIsOccupying)
    {
      outputManager->SimultaneousTxOtherTechnology (nodeId);
      std::cout << "There was a sim tx from other tech nr" << std::endl;
    }
  if (m_wifiIsOccupying)
    {
      outputManager->SimultaneousTxSameTechnology (nodeId);
      std::cout << "There was a sim tx from same tech wifi" << std::endl;
    }

  m_wifiOccupancy += time;
  m_wifiIsOccupying = true;
  Simulator::Schedule (time, &ResetWifiOccupancy);
}



int
main (int argc, char *argv[])
{
  bool cellScan = true;
  double beamSearchAngleStep = 30.0; // degrees
  double totalTxPower = 4; // dBm
  double ueTxPower = 2; // dBm
  uint16_t numerologyBwp1 = 2; //FIXME: Negative delay in NrPhy::GetSymbolPeriod with numerology = 1...
  double frequencyBwp1 = 6.615e9; // Hz Wi-Fi channel 42
  double bandwidthBwp1 = 80e6; // Hz
  double ueX = 5.0; // meters

  double simTime = 1.5; // seconds
  double udpAppStartTime = 0.4; //seconds
  uint32_t scenarioId = 0;
  uint32_t runId = 0;
  uint32_t seed = 1;
  bool enableNr = true;
  bool enableWifi = true;
  bool doubleTechnology = false;
  double cat2EDThreshold = -69.0; // dBm
  double cat3and4EDThreshold = -79.0; // dBm
  uint32_t gnbCount = 3, ueCount = 4;

  std::cout << "After vars" << std::endl;

  std::string rlcModel = "RlcUmAlways";
  std::string errorModel = "ns3::NrEesmCcT1";
  std::string nodeRate = "10000000bps";
  std::string gnbCamType = "ns3::NrCat4LbtAccessManager";
  std::string ueCamType = "ns3::NrAlwaysOnAccessManager"; //this should be kept always on since grant-free UL is not supported yet
  std::string wifiStandard = "11ax";

  CommandLine cmd;

  cmd.AddValue ("simTime", "Simulation time (seconds)", simTime);
  cmd.AddValue ("cellScan",
                "Use beam search method to determine beamforming vector,"
                " the default is long-term covariance matrix method"
                " true to use cell scanning method, false to use the default"
                " power method.",
                cellScan);
  cmd.AddValue ("beamSearchAngleStep",
                "Beam search angle step (degrees) for beam search method",
                beamSearchAngleStep);
  cmd.AddValue ("totalTxPower",
                "Total TX power (dBm) that will be proportionally assigned to"
                " bandwidth parts depending on each BWP bandwidth ",
                totalTxPower);
  cmd.AddValue ("errorModelType",
                "Error model type: ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1, ns3::NrEesmIrT2",
                errorModel);
  cmd.AddValue ("rlcModel", "The NR RLC Model: RlcTmAlways or RlcUmAlways", rlcModel);
  cmd.AddValue ("ueX", "X position (meters) of any UE", ueX);
  cmd.AddValue ("scenario",
                "Scenario (0 = simple interference, 1 = new position)",
                scenarioId);
  cmd.AddValue ("seed", "Simulation seed", seed);
  cmd.AddValue ("runId", "Simulation Run ID", runId);
  cmd.AddValue ("enableNr", "Enable NR node", enableNr);
  cmd.AddValue ("enableWifi", "Enable Wi-Fi nodes", enableWifi);
  cmd.AddValue ("gnbCount", "Number of gnb nodes (will be doubled)", gnbCount);
  cmd.AddValue ("ueCount", "Number of ue nodes (will also be doubled)", ueCount);
  cmd.AddValue ("nodeRate", "The rate of every node in the network", nodeRate);
  cmd.AddValue ("gnbCamType", "The gNB CAM", gnbCamType);
  cmd.AddValue ("doubleTechnology", "Double the technology", doubleTechnology);
  cmd.AddValue ("cat2EDThreshold", "The ED threshold to be used by Lbt category 2 algorithm (dBm). Allowed range [-100.0, 0.0]., ", cat2EDThreshold);
  cmd.AddValue ("cat3and4EDTreshold", "The ED threshold to be used by Lbt category 3 and 4 algorithm (dBm). Allowed range [-100.0, 0.0].", cat3and4EDThreshold);
  cmd.AddValue ("wifiStandard", "The Wi-Fi standard to use (11ac, 11ax).", wifiStandard);
  cmd.AddValue ("nrNumerology", "The numerology to use for NR devices (determines subcarrier spacing, symbols per slott, and number of resource blocks)", numerologyBwp1);

  cmd.Parse (argc, argv);

  Time::SetResolution (Time::NS);

  RngSeedManager::SetSeed (seed);
  RngSeedManager::SetRun (runId);
  ConfigureDefaultValues (cellScan, beamSearchAngleStep, errorModel, cat2EDThreshold, cat3and4EDThreshold, rlcModel);

  //LogComponentEnable("NrPhy", LOG_ALL);
  //LogComponentEnable("WifiPhy", LOG_ALL);

  //std::fstream file;
  //file.open("cout.txt", std::ios::out);
  //std::string line;

  //std::streambuf* stream_buffer_file = file.rdbuf();
  //std::cout.rdbuf(stream_buffer_file);
  

  std::cout << "after cmd line" << std::endl;

  enum WifiStandard standard = WIFI_STANDARD_80211ax_6GHZ;
  if (wifiStandard == "11ac")
    {
      standard = WIFI_STANDARD_80211ac;
    }
  else if (wifiStandard != "11ax")
    {
      NS_ABORT_MSG ("Unsupported Wi-Fi standard");
    }

  //NodeDistributionScenario *scenario;
  std::unique_ptr<HalfMatrixLayout> operatorALayout, operatorBLayout;
  ns3::Vector maxArea = {60, 20, 0};

  // Place of the node inside the UE/STA container

  NS_ASSERT (!(enableNr && enableWifi && doubleTechnology));
  std::cout << "after some assert" << std::endl;
  /*
  //if (scenarioId == 0)
    //{
      //scenario = new SinglePairNodeScenario (nodes, Vector (0, 0, 1.5), ueX);
      //scenario = new DenseNodeScenario (2 * gnbCount, 2 * ueCount);
    //}
  else if (scenarioId == 1)
    {
      NS_ASSERT (doubleTechnology || (enableNr && enableWifi));
      scenario = new InFrontNodeDistribution (Vector (0, 0, 1.5), 10.0, ueX);
    }
  else if (scenarioId == 2)
  {
    scenario = new DenseNodeScenario (gnbCount, ueCount);
  }
  else
    {
      NS_FATAL_ERROR ("Scenario not recognized");
    }
  */

  std::cout << "after scinerio" << std::endl;

  std::stringstream ss;
  std::string technology = "";
  std::string gnbCam = "";
  if (enableNr)
    {
      technology += "with-nr-";
    }
  else
    {
      technology += "without-nr-";
    }
  if (enableWifi)
    {
      technology += "with-wifi-" + wifiStandard + "-";
    }
  else
    {
      technology += "without-wifi-";
    }

  if (doubleTechnology)
    {
      technology += "nr-wifi-" + wifiStandard + "-";
    }
  if (gnbCamType == "ns3::NrCat4LbtAccessManager")
    {
      gnbCam = "Cat4Lbt";
    }
  else if (gnbCamType == "ns3::NrCat3LbtAccessManager")
    {
      gnbCam = "Cat3Lbt";
    }
  else if (gnbCamType == "ns3::NrCat2LbtAccessManager")
    {
      gnbCam = "Cat2Lbt";
    }
  else if (gnbCamType == "ns3::NrOnOffAccessManager")
    {
      gnbCam = "OnOff";
    }
  else if (gnbCamType == "ns3::NrAlwaysOnAccessManager")
    {
      gnbCam = "AlwaysOn";
    }
  else
    {
      gnbCam = "Unknown";
      NS_ABORT_MSG ("Unknown gnbCamType: " << gnbCamType);
    }

  Packet::EnablePrinting ();

  ss << technology << "-ueC-" << ueCount << "-nrN-" << numerologyBwp1 << "-rate-" << nodeRate;
  std::cout << "post scenario " << std::endl;

  FileOutputManager manager (ss.str ());
  outputManager = &manager;

  //L2Setup *nr, *wifi;
  L2Setup *nr;
  std::vector<std::unique_ptr<WifiSetup>> wifi;

  // Will be instantiated and configured by NrSingleBwpSetup
  Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel> ();
  Ptr<ThreeGppPropagationLossModel> propagation = CreateObject<ThreeGppIndoorOfficePropagationLossModel> ();
  //Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  Ptr<ThreeGppSpectrumPropagationLossModel> spectrumPropagation = CreateObject<ThreeGppSpectrumPropagationLossModel> ();

  //channel->SetPropagationDelayModel (delayModel);

  propagation->SetAttributeFailSafe ("Frequency", DoubleValue (frequencyBwp1));
  spectrumPropagation->SetChannelModelAttribute ("Frequency", DoubleValue (frequencyBwp1));

  BandwidthPartInfo::Scenario channelScenario = BandwidthPartInfo::InH_OfficeMixed;
  Ptr<ChannelConditionModel> channelConditionModel = CreateObject<ThreeGppIndoorMixedOfficeChannelConditionModel> ();
  spectrumPropagation->SetChannelModelAttribute ("Scenario", StringValue ("InH-OfficeMixed"));
  spectrumPropagation->SetChannelModelAttribute ("ChannelConditionModel", PointerValue (channelConditionModel));
  propagation->SetChannelConditionModel (channelConditionModel);

  channel->AddPropagationLossModel (propagation);
  channel->AddSpectrumPropagationLossModel (spectrumPropagation);

  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod",TimeValue (NanoSeconds (0)));
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  std::stringstream ssRemote;
  ssRemote << "REMOTE-" << remoteHost->GetId ();
  Names::Add (ssRemote.str(), remoteHost);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  Ipv4InterfaceContainer ueIpIfaceOperatorA, ueIpIfaceOperatorB;
  //Ipv4InterfaceContainer ueIpIface;
  bool operatorAUsed = false;

  std::cout << "After ip setup" << std::endl;
  
  //NodeContainer gnbs;
  //NodeContainer ues;
  //NodeContainer gnbsA;
  //NodeContainer gnbsB;
  //NodeContainer uesA;
  //NodeContainer uesB;

  /*  
  gnbs = NodeContainer (scenario->GetGnbs ());
  ues = NodeContainer (scenario->GetUes ());
  PrintScenario(gnbCount, ueCount);
  
  NodeContainer::Iterator i;
  for (i = gnbs.Begin (); i != gnbs.End (); ++i)
  {
    if (gnbsA.GetN () < gnbCount) 
    {
      gnbsA.Add(*i);
    } else {
      gnbsB.Add(*i);
    }
  }
  
  for (i = ues.Begin (); i != ues.End (); ++i)
  {
    if (uesA.GetN () < ueCount) 
    {
      uesA.Add(*i);
    } else {
      uesB.Add(*i);
    }
  }
  */

  //std::cout << "gNbsA: " << gnbsA.GetN () << " gNbsB: " << gnbsB.GetN () << std::endl;
  //std::cout << "uesA: " << uesA.GetN () << " uesB: " << uesB.GetN () << std::endl;

  if (enableNr)
    {
      std::unique_ptr<Ipv4AddressHelper> address = std::unique_ptr<Ipv4AddressHelper> (new Ipv4AddressHelper ());
      Ipv4InterfaceContainer totalContainer;
      NodeContainer gnbsC;
      NodeContainer uesC;
      std::unordered_map<uint32_t, uint32_t> connections;

      operatorALayout = std::unique_ptr<HalfMatrixLayout> (new HalfMatrixLayout(2, gnbCount, ueCount, ns3::Vector (0,0,0), maxArea,
                                                                                  ns3::HalfMatrixLayout::TOP, 0.0, 0.0));

      gnbsC.Add (operatorALayout->GetGnbs ());
      uesC.Add (operatorALayout->GetUes ());
      operatorAUsed = true;

      if (doubleTechnology)
        {
          operatorBLayout = std::unique_ptr<HalfMatrixLayout> (new HalfMatrixLayout(2, gnbCount, ueCount, ns3::Vector (0,0,0), maxArea,
                                                                                  ns3::HalfMatrixLayout::BOTTOM, 0.0, 0.0));
          gnbsC.Add (operatorBLayout->GetGnbs ());
          uesC.Add (operatorBLayout->GetUes ());
          //connections = { { 0, 0}, {1, 1}};
        }
      for (auto it = gnbsC.Begin(); it != gnbsC.End(); ++it)
      {
        std::cout << "Gnb " << (*it)->GetId() << std::endl;
      }
      for (auto it = uesC.Begin(); it != uesC.End(); ++it)
      {
        std::cout << "Ue " << (*it)->GetId() << std::endl;
      }

      nr = new NrSingleBwpSetup (gnbsC,uesC, channel, propagation, spectrumPropagation,
                                 frequencyBwp1, bandwidthBwp1, numerologyBwp1, totalTxPower, ueTxPower, connections,
                                 gnbCamType, ueCamType, "ns3::NrMacSchedulerTdmaPF", channelScenario);
      totalContainer.Add (nr->AssignIpv4ToUe (address));
      //ueIpIface.Add(totalContainer);
      nr->AssignIpv4ToStations (address); // Not used
      nr->ConnectToRemotes (remoteHostContainer, "1.0.0.0");
      nr->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
      nr->SetMacTxDataFailedCb (MakeCallback (&OutputManager::MacDataTxFailed, &manager));
      nr->SetChannelOccupancyCallback (MakeCallback (&NrOccupancy));
      if (doubleTechnology)
      {
        for (auto itIp = totalContainer.Begin (); itIp != totalContainer.End (); ++itIp)
        {
          bool found = false;
          for (auto it = operatorBLayout->GetUes().Begin(); it != operatorBLayout->GetUes().End(); ++it)
          {
            NS_ASSERT ((*itIp).first->GetNInterfaces() > 0);

            if ((*itIp).first->GetNetDevice(0)->GetNode()->GetId () == (*it)->GetId ())
            {
              found = true;
              break;
            }
          }
          if (found)
          {
            ueIpIfaceOperatorB.Add (*itIp);
          }
          else
          {
            ueIpIfaceOperatorA.Add(*itIp);
          }
        }
      }
      else
      {
        ueIpIfaceOperatorA.Add (totalContainer);
      }
    }

  std::cout << "NR established" << std::endl;
  std::unordered_map<uint32_t, NodeContainer> networkWifiMap;
  std::unordered_map<uint32_t, Ptr<Node>> apMap;
  if (enableWifi)
    {
      NodeContainer gnbsC;
      NodeContainer uesC;

      if (operatorAUsed)
      {
        NS_ASSERT(operatorBLayout == nullptr);
        operatorBLayout = std::unique_ptr<HalfMatrixLayout> (new HalfMatrixLayout(2, gnbCount, ueCount, ns3::Vector (0,0,0), maxArea,
                                                                                      ns3::HalfMatrixLayout::BOTTOM, 0.0, 0.0));
        gnbsC.Add (operatorBLayout->GetGnbs ());
        uesC.Add (operatorBLayout->GetUes ());
      }
      else
      {
        NS_ASSERT(operatorALayout == nullptr);
        operatorALayout = std::unique_ptr<HalfMatrixLayout> (new HalfMatrixLayout(2, gnbCount, ueCount, ns3::Vector (0,0,0), maxArea,
                                                                                      ns3::HalfMatrixLayout::TOP, 0.0, 0.0));
 
        gnbsC.Add (operatorALayout->GetGnbs ());
        uesC.Add (operatorALayout->GetUes ());
      }

      if (doubleTechnology)
      {
        operatorBLayout = std::unique_ptr<HalfMatrixLayout> (new HalfMatrixLayout(2, gnbCount, ueCount, ns3::Vector (0,0,0), maxArea,
                                                                                      ns3::HalfMatrixLayout::BOTTOM, 0.0, 0.0));
        gnbsC.Add (operatorBLayout->GetGnbs());
        uesC.Add (operatorBLayout->GetUes());
      }

      for (auto it = gnbsC.Begin(); it != gnbsC.End(); ++it)
      {
        std::cout << "Gnb " << (*it)->GetId() << std::endl;
      }
      for (auto it = uesC.Begin(); it != uesC.End(); ++it)
      {
        std::cout << "Ue " << (*it)->GetId() << std::endl;
      }

      for (auto it = uesC.Begin(); it != uesC.End(); ++it)
      {
        Ptr<Node> closestAp = GetClosestAp (*it, gnbsC);
        uint32_t closestApId = closestAp->GetId();
        std::cout << (*it)->GetId() << " Ue connected with Gnb: " << closestAp->GetId() << std::endl;
        auto v = networkWifiMap.find (closestApId);
        if (v == networkWifiMap.end ())
        {
          v = networkWifiMap.emplace (std::make_pair (closestApId, NodeContainer ())).first;
        }
        v->second.Add (*it);

        if (apMap.find (closestApId) == apMap.end ())
        {
          apMap.insert (std::make_pair (closestApId, closestAp));
        }
      }

      
      uint32_t startingClass = 1;
      for (const auto & v : networkWifiMap)
      {
        //uint32_t closestApId = closestAp->GetId();

        std::stringstream base, ssid, remoteAddr;
        base << "10." << startingClass << ".0.0";
        remoteAddr << "2." << startingClass << ".0.0";
        ssid << "coex" << startingClass;
        startingClass++;

        auto it = apMap.find (v.first);
        std::cout << "it->second id: " << (it->second)->GetId() << " v.second num: " << v.second.GetN() << std::endl;

        std::unique_ptr<Ipv4AddressHelper> address = std::unique_ptr<Ipv4AddressHelper> (new Ipv4AddressHelper ());
        address->SetBase(Ipv4Address (base.str().c_str ()), "255.255.255.0");
        //auto it = apMap.find (v.first);
        wifi.push_back (std::unique_ptr<WifiSetup> (new WifiSetup (NodeContainer (it->second), v.second,
                              channel, propagation, spectrumPropagation,
                              frequencyBwp1, bandwidthBwp1, totalTxPower, totalTxPower, -62.0, -62.0,
                              standard, ssid.str())));

        Ipv4InterfaceContainer devIface = wifi.back()->AssignIpv4ToUe (address);
        //ueIpIface.Add(devIface);
        if (operatorAUsed)
        {
          ueIpIfaceOperatorB.Add (devIface);
        } else {
          ueIpIfaceOperatorA.Add (devIface);
        }
        wifi.back()->AssignIpv4ToStations (address);
        wifi.back()->ConnectToRemotes (remoteHostContainer, remoteAddr.str().c_str());
        wifi.back()->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
        wifi.back()->SetMacTxDataFailedCb (MakeCallback (&OutputManager::MacDataTxFailed, &manager));
        wifi.back()->SetChannelOccupancyCallback (MakeCallback (&WifiOccupancy));
        wifi.back()->SetChannelRTACallback (MakeCallback (&OutputManager::ChannelRequestTime, &manager));
      }
      /*
      if (doubleTechnology)
        {
          address->SetBase ("10.0.1.0", "255.255.255.0");
          auto secondWifi = new WifiSetup (gnbsB,
                                           uesB,
                                           channel, propagation, spectrumPropagation,
                                           frequencyBwp1, bandwidthBwp1, totalTxPower, totalTxPower,
                                           -62.0, -62.0,
                                           standard, "segundo");
          std::cout << "after second" << std::endl;
          ueIpIface.Add (secondWifi->AssignIpv4ToUe (address));
          secondWifi->AssignIpv4ToStations (address);
          secondWifi->ConnectToRemotes (remoteHostContainer, "2.0.1.0");
          secondWifi->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
          secondWifi->SetMacTxDataFailedCb (MakeCallback (&OutputManager::MacDataTxFailed, &manager));
          secondWifi->SetChannelOccupancyCallback (MakeCallback (&WifiOccupancy));
          dynamic_cast<WifiSetup*> (secondWifi)->SetChannelRTACallback (MakeCallback (&OutputManager::ChannelRequestTime, &manager));
        }
      */  
  }

  std::cout << "after wifi" << std::endl;
  uint32_t ifId = 1;
  if (enableNr)
    {
      Ipv4StaticRoutingHelper ipv4RoutingHelper;
      for (auto it = remoteHostContainer.Begin (); it != remoteHostContainer.End (); ++it)
        {
          Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting ((*it)->GetObject<Ipv4> ());
          remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), ifId);
        }
      ifId++;
    }

  if (enableWifi)
  {
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (auto it = remoteHostContainer.Begin (); it != remoteHostContainer.End (); ++it)
    {
      for (uint32_t startingClass = 1; startingClass <= networkWifiMap.size (); startingClass++)
      {
        std::stringstream base;
        base << "10." << startingClass << ".0.0";
        Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting ((*it)->GetObject<Ipv4> ());
        remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address (base.str().c_str()), Ipv4Mask ("255.255.255.0"), ifId);
        ifId++;
      }
    }
  }

  std::cout << "after nodes are set" << std::endl;

  uint16_t dlPortA = 1233;
  uint16_t dlPortB = 1234;
  ApplicationContainer clientApps, serverApps;
      
  if (operatorALayout != nullptr)
  {
    PacketSinkHelper dlPacketSinkHelperA ("ns3::UdpSocketFactory",
        Address (InetSocketAddress (Ipv4Address::GetAny (), dlPortA)));
    dlPacketSinkHelperA.SetAttribute ("EnableSeqTsSizeHeader", BooleanValue (true));
    //dlPacketSinkHelperA.SetAttribute("EnableE2EStats", BooleanValue (true));
    serverApps.Add (dlPacketSinkHelperA.Install (operatorALayout->GetUes()));

    for (uint32_t j = 0; j < ueIpIfaceOperatorA.GetN(); ++j)
    {
      /*
      Address ueAddress = ueIpIfaceOperatorA.GetAddress (j);

      UdpClientHelper dlClient;;
      dlClient.SetAttribute ("RemotePort", UintegerValue (dlPortA));
      dlClient.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));
      dlClient.SetAttribute ("PacketSize", UintegerValue (PACKET_SIZE));
      dlClient.SetAttribute ("Interval", TimeValue (Seconds (1.0 / 10000)));
      dlClient.SetAttribute ("RemoteAddress", AddressValue (ueAddress));
      */
      OnOffHelper onoff ("ns3::UdpSocketFactory",
          Address (InetSocketAddress (ueIpIfaceOperatorA.GetAddress (j), dlPortA)));
      onoff.SetConstantRate (DataRate (nodeRate), PACKET_SIZE);
      onoff.SetAttribute ("EnableSeqTsSizeHeader", BooleanValue (true));
      //onoff.SetAttribute ("EnableE2EStats", BooleanValue (true));
      ApplicationContainer app = onoff.Install (remoteHostContainer);
      app.Start (Seconds(udpAppStartTime));
      app.Stop (Seconds(udpAppStartTime + 1.0));
      clientApps.Add (app);
      std::cout << "Installing app to transmit data to the operator A node "
                      << ueIpIfaceOperatorA.GetAddress (j)
                      << ":" << dlPortA << " at time " << udpAppStartTime
                      << " s and stop at " << udpAppStartTime + 1.0 << " s." << std::endl;
    }
  }
  

  if (operatorBLayout != nullptr)
  {
    PacketSinkHelper dlPacketSinkHelperB ("ns3::UdpSocketFactory",
        Address (InetSocketAddress (Ipv4Address::GetAny (), dlPortB)));
    dlPacketSinkHelperB.SetAttribute ("EnableSeqTsSizeHeader", BooleanValue (true));
    //dlPacketSinkHelperB.SetAttribute("EnableE2EStats", BooleanValue (true));
    serverApps.Add (dlPacketSinkHelperB.Install (operatorBLayout->GetUes()));


    for (uint32_t j = 0; j < ueIpIfaceOperatorB.GetN (); ++j)
    {
      /*
      Address ueAddress = ueIpIfaceOperatorB.GetAddress (j);
      
      UdpClientHelper dlClient;;
      dlClient.SetAttribute ("RemotePort", UintegerValue (dlPortB));
      dlClient.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));
      dlClient.SetAttribute ("PacketSize", UintegerValue (PACKET_SIZE));
      dlClient.SetAttribute ("Interval", TimeValue (Seconds (1.0 / 10000)));
      dlClient.SetAttribute ("RemoteAddress", AddressValue (ueAddress));
      */
      OnOffHelper onoff ("ns3::UdpSocketFactory",
          Address (InetSocketAddress (ueIpIfaceOperatorB.GetAddress (j), dlPortB)));
      onoff.SetConstantRate (DataRate (nodeRate), PACKET_SIZE);
      onoff.SetAttribute ("EnableSeqTsSizeHeader", BooleanValue (true));
      //onoff.SetAttribute ("EnableE2EStats", BooleanValue (true));
      ApplicationContainer app = onoff.Install (remoteHostContainer);
      app.Start (Seconds (udpAppStartTime));
      app.Stop (Seconds (udpAppStartTime + 1.0));
      clientApps.Add (app);
      std::cout << "Installing app to transmit data to the operator B node "
                      << ueIpIfaceOperatorB.GetAddress (j)
                      << ":" << dlPortB << " at time " << udpAppStartTime
                      << " s and stop at " << udpAppStartTime + 1.0 << " s." << std::endl;
    }
  }

  serverApps.Start (Seconds (udpAppStartTime));
  serverApps.Stop (Seconds (simTime));

  PopulateArpCache ();

  if (operatorALayout != nullptr)
  {
    std::cout << "############# OPERATOR A" << std::endl;
    std::cout << "UE: " << std::endl;
    PrintIpAddress (operatorALayout->GetUes());
    PrintRoutingTable (operatorALayout->GetUes());
    std::cout << "GNB" << std::endl;
    PrintIpAddress (operatorALayout->GetGnbs());
    PrintRoutingTable (operatorALayout->GetGnbs());
    std::stringstream n, m;
    n << "gnb-lay-a-" << RngSeedManager::GetRun () << "-ueC-" << ueCount << "-nrN-"<< numerologyBwp1 << ".data";
    operatorALayout->PrintGnbPosToFile(n.str());
    m << "ue-lay-a-" << RngSeedManager::GetRun () << "-ueC-" << ueCount << "-nrN-"<< numerologyBwp1 << ".data";
    operatorALayout->PrintUePosToFile (m.str());
  }

  if (operatorBLayout != nullptr)
  {
    std::cout << "############# OPERATOR B" << std::endl;
    std::cout << "UE: " << std::endl;
    PrintIpAddress (operatorBLayout->GetUes());
    PrintRoutingTable (operatorBLayout->GetUes());
    std::cout << "GNB" << std::endl;
    PrintIpAddress (operatorBLayout->GetGnbs());
    PrintRoutingTable (operatorBLayout->GetGnbs());
    std::stringstream n, m;
    n << "gnb-lay-b-" << RngSeedManager::GetRun () << "-ueC-" << ueCount << "-nrN-"<< numerologyBwp1 << ".data";
    operatorBLayout->PrintGnbPosToFile(n.str());
    m << "ue-lay-b-" << RngSeedManager::GetRun () << "-ueC-" << ueCount << "-nrN-"<< numerologyBwp1 << ".data";
    operatorBLayout->PrintUePosToFile (m.str());
  }

  std::cout << "REMOTE" << std::endl;
  PrintIpAddress (remoteHostContainer);
  PrintRoutingTable (remoteHostContainer);

  /*
  for (auto itGnb = gnbs.Begin(); itGnb != gnbs.End(); itGnb++)
  {
    Vector v = (*itGnb)->GetObject<MobilityModel>()->GetPosition();
    std::cout << (*itGnb)->GetId() << " " << v.x << " " << v.y << " " << v.z << std::endl;
  }
  
  for (auto itUe = ues.Begin(); itUe != ues.End(); itUe++)
  {
    Vector v = (*itUe)->GetObject<MobilityModel>()->GetPosition();
    std::cout << (*itUe)->GetId() << " " << v.x << " " << v.y << " " << v.z << std::endl;
  }
  */

  std::cout << "Things ar about to run" << std::endl;

  Simulator::Schedule (MicroSeconds (100), &TimePasses);

  // Flow monitor
  FlowMonitorHelper flowHelper;
  auto monitor = flowHelper.InstallAll ();

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowHelper.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
  {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
    bool isNr = false, isWigig = false;
    if (enableNr)
    {
      if (t.destinationPort == dlPortA)
      {
        isNr = true;
      }
      if (doubleTechnology && t.destinationPort == dlPortB)
      {
        isNr = true;
      }
    }
    if (enableWifi)
    {
      if (!enableNr && t.destinationPort == dlPortA)
      {
        isWigig = true;
      }
      else if (t.destinationPort == dlPortB)
      {
        isWigig = true;
      }
      if (doubleTechnology && t.destinationPort == dlPortA)
      {
        isWigig = true;
      }
    }

    //bool isNr = (t.destinationPort == dlPortA || (t.destinationPort == dlPortB && doubleTechnology));
    //bool isWigig = (t.destinationPort == dlPortB || (t.destinationPort == dlPortA && doubleTechnology));
    std::cout << "Des port " << t.destinationPort << " Source port " << t.sourcePort<< std::endl;
    std::cout << "Des address " << t.destinationAddress <<  " Source address" << t.sourceAddress << std::endl;
    if (isNr || isWigig)
    {
      std::string technology = isNr ? "nr" : "wifi";
      double thMbps = i->second.rxBytes * 8.0 / (simTime - udpAppStartTime) / 1e6;
      double delay = 0.0;
      double jitter = 0.0;
      if (i->second.rxPackets > 1)
      {
        delay = i->second.delaySum.GetMicroSeconds () / i->second.rxPackets;
        jitter = i->second.jitterSum.GetMicroSeconds () / (i->second.rxPackets - 1);
      }
      std::stringstream addr;
      t.destinationAddress.Print (addr);
      if (i->second.txBytes > 300)
      {
        manager.StoreE2EStatsFor (technology, thMbps, i->second.txBytes, i->second.rxBytes,
                                        delay, jitter, addr.str ());
        std::cout << "Technology " << technology << "\nthMbs " << thMbps << "\ntxBytes " << i->second.txBytes << "\nrxBytes " << i->second.rxBytes << "\ndelay " << delay << "\njitter " << jitter << "\naddr " << addr.str() << "\nLost packets: " << i->second.lostPackets << "\n" << std::endl;
      }
    }
  }

  if (enableNr)
  {
      manager.StoreChannelOccupancyRateFor ("nr", m_nrOccupancy.GetSeconds () / (simTime - udpAppStartTime));
      std::cout << "nrOccupancy: " << m_nrOccupancy.GetSeconds () / (simTime - udpAppStartTime) << std::endl;
    }

  if (enableWifi)
    {
      manager.StoreChannelOccupancyRateFor ("wifi", m_wifiOccupancy.GetSeconds () / (simTime - udpAppStartTime));
      std::cout << "wifiOccupancy: " << m_wifiOccupancy.GetSeconds () / (simTime - udpAppStartTime) << std::endl;
    }

  manager.Close ();


  Simulator::Destroy ();
  return 0;
}

