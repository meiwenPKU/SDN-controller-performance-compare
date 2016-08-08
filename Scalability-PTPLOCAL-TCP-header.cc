/*
 * This program is used to simulate the flow installation and statistics gathering process in the
 * P2P SDN. each controller only has the local vision of the network it controlled.
 * For every initial flow setup, it will go through a controller with a probability
 *
 * Every switch and Controller will be binded with one send socket and one receive socket.
 * For the switch, it will send the initial flow installation packet to the Controller when a flow arrives.
 * And install the flow entry in the flow table when received a flow install packet from the Controller.
 * For the Controller, it will periodically send FeatureRequest packet to the switch to gather the statistics,
 * once the switch received the packet, it will send statics back to the Controller with a delay proportional to
 * the size of flow table. And when the Controller once receive the initial flow installation packet from the
 * switch, the packet will be put into a FIFO. And for every packet in the FIFO, the Controller will respond it
 * with a delay proportional to the size of the network it controls.
 *
 *
 * notes: 1) the size of the packet is derived from openflow 1.3.0, the size of different packets
 *           should be different because in this program we use the size of the packet to identify the type of the packet
 *        2) ECHO request and response are neglected in this study
 *        3) we assume the time for response an initial flow setup packet is determined by Cg(V,E)
 *           where C is an constant representing the CPU speed (e.g., Hz), g(V,E) is the time complexity to
 *           run Dijkstra algorithm in the graph (V,E)
 *        4) the time for pull the statistic is proportional to the size of the flow table, i.e. K*S
 *           where K reflects the cpu speed in the switch, S is the size of the table
 *        5) we do not simulate the process of finding the shortest path. Instead, we only cares how many switches
 *           are in the route. Therefore, we use the diameter of the graph to approximate the number of switches
 *           in one route.
 *        6) the lifetime of the flow flows to a distribution (e.g., uniform, bernoulli, binomial, poisson, etc). When a
 *           flow is finished, a new flow is then generated. Namely, there is no interval between flows.
 *
 */


#include <fstream>
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;
enum PacketType {FlowInitial, FlowMod,FlowRemove,StatsReq, StatsRes};
//-------------define the packet size---------------------
uint32_t SIZE_PKT_INITIAL_FLOW = 36; // the size of PacketIn message
uint32_t SIZE_PKT_FLOW_MOD = 56; // the size of FlowMod message
uint32_t SIZE_PKT_FLOW_REMOVE = 52; // the size of FlowRemove message, it is optional
/*
 * the size of MultipartReq message,for different features(flow, port, queue, etc), the size is different:
 * flow(32), aggregate(32), port(8), queue(8), group(8), meter(8),
 * meterConfig(8), tableFeatures(44), experimenter(12)
 */
 uint32_t SIZE_PKT_STATS_REQ = 16+8;
/*
 * the size of MultipartRes message, for different features(flow, port, queue, etc), the size is different:
 * Desc (2120), Flow (48), Aggregate(24), Table(60), Port(104), Queue(32), Group(48), GroupDesc(8),
 * GroupFeature(40), experimenter(8)
 */
 uint32_t SIZE_PKT_STATS_RES = 16+60;

//--------------define the processing time of the Controller-------------------
 int C = 10; // the unit is nanosecond, refer to "distribution of execution times for sorting algorithms"

//--------------define the time of pulling the statistics from the switch------
 int K = 1200; // the unit is micro second

//--------------define the hard expire time for the flow----------------------------
int timeout = 10; // unit is seconds

//---------------define the time interval for polling the timeout flow
int pollInterval = 1000; // unit is milli second

//--------------define the period for pulling the statistics------------------------
 int pullInterval = 5; // unit is seconds


 int numVertices = 2; // the number of vertices in the network
 //int Diameter = log((numVertices - 1)*(degree-2)/degree +1) / log(degree-1); // apply the degree diameter problem in wikipedia
 double Diameter = 2; // the average number of switches a flow going through in one domain
 int numControllers = 3; //  the number of controllers in the network
 double Diameter_controller = 5; // the average number of controllers a flow going through


 std::string ControlDataRate = "260Kbps";
 std::string ControlDelay = "1ms";
 uint32_t N_flow = 1000000; // the number of flows generated by each switch
 double aver_interval = 50; // the average interval (exponential distribution)

 int simulationTime = 300;
 const int OFCONTROLLERPORT = 4477;
 const int OFSWITCHPORT = 9999;

NS_LOG_COMPONENT_DEFINE ("P2PLocalSDN");

class MyHeader : public Header
{
public:

  MyHeader ();
  virtual ~MyHeader ();

  void SetData (uint16_t PacketType, uint16_t SenderID, uint32_t PacketID);
  uint16_t GetPacketType (void) const;
  uint16_t GetSenderID (void) const;
  uint32_t GetPacketID (void) const;


  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual uint32_t GetSerializedSize (void) const;
private:
  uint16_t m_PacketType;
  uint16_t m_SenderID;
  uint32_t m_PacketID;
};

MyHeader::MyHeader ()
{
  // we must provide a public default constructor,
  // implicit or explicit, but never private.
}
MyHeader::~MyHeader ()
{
}

TypeId
MyHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MyHeader")
    .SetParent<Header> ()
    .AddConstructor<MyHeader> ()
  ;
  return tid;
}
TypeId
MyHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void
MyHeader::Print (std::ostream &os) const
{
  // This method is invoked by the packet printing
  // routines to print the content of my header.
  //os << "data=" << m_data << std::endl;
  os << "Packet type =" << m_PacketType << "; Packet ID = " << m_PacketID << "; Sender ID = " << m_SenderID;
}
uint32_t
MyHeader::GetSerializedSize (void) const
{
  // we reserve 6 bytes for our header.
  return 8;
}
void
MyHeader::Serialize (Buffer::Iterator start) const
{
  // we can serialize two bytes at the start of the buffer.
  // we write them in network byte order.
  start.WriteHtonU16(m_PacketType);
  start.WriteHtonU16(m_SenderID);
  start.WriteHtonU32(m_PacketID);
}
uint32_t
MyHeader::Deserialize (Buffer::Iterator start)
{
  // we can deserialize two bytes from the start of the buffer.
  // we read them in network byte order and store them
  // in host byte order.
  m_PacketType = start.ReadNtohU16 ();
  m_SenderID = start.ReadNtohU16 ();
  m_PacketID = start.ReadNtohU32();
  // we return the number of bytes effectively read.
  return 8;
}

void
MyHeader::SetData (uint16_t PacketType, uint16_t SenderID, uint32_t PacketID)
{
  m_PacketType = PacketType;
  m_SenderID = SenderID;
  m_PacketID = PacketID;
}
uint16_t
MyHeader::GetPacketType (void) const
{
  return m_PacketType;
}

uint32_t
MyHeader::GetPacketID(void) const

{
	return m_PacketID;
}

uint16_t
MyHeader::GetSenderID(void) const
{
	return m_SenderID;
}

class Switch
{
public:
	uint32_t m_numFlow; // the number of flow entries in the flow table
	std::list<Time> m_vInsertTime; // the vector of the setup time for each flow
	Ptr<Socket> m_SendSocket; // the send socket between the Controller and the switch
	Ptr<Socket> m_ReceiveSocket; // the receive socket between the Controller and the switch
	std::vector<double> m_vDelay; // the vector of the flow setup delay for each flow

	uint32_t m_id; // the id of the switch
	uint32_t m_num_flow_setup; // the number of flow setup packets sent
	uint32_t m_num_flow_remove; // the number of flow remove packets sent
	uint32_t m_num_status_report; //the number of status report packets sent
	uint32_t m_num_flow_mod; // the number of flow modification packets received
	uint32_t m_num_status_request; // the number of status request packets received

	//std::queue<uint32_t> m_QSentFlowInitial; // the queue of the flow initial packet id
	//std::queue<double> m_QLastSend; // the queue of the time the flow initial packet is sent
	std::map<uint32_t,double> m_mFlowInitial;
	uint32_t m_id_flow_initial; // the id of the flow initial packet
	std::vector<uint32_t> m_vNumFlow; // the vector of the number of flows in the flow table

public:
	Switch():m_numFlow(0),m_id(0), m_num_flow_setup (0), m_num_flow_remove (0), m_num_status_report(0),
	m_num_flow_mod(0), m_num_status_request(0), m_id_flow_initial(1){

	}
	/*
	 * \brief Send the packet (initial flow setup packet, flow remove, status response)
	 * to the Controller from the switch
	 * \param pktSize, the size of the packet
	 */
	void SendPacket (uint16_t pktType)
	{
		if (pktType == FlowInitial)
		{
			NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Switch " << m_id << " Send a initial flow setup packet!");
			//m_QLastSend.push(Simulator::Now().GetSeconds());
			//m_QSentFlowInitial.push(m_id_flow_initial);
			m_mFlowInitial.insert(std::pair<uint32_t,double>(m_id_flow_initial,Simulator::Now().GetSeconds()));
		    MyHeader sourceHeader;
		    sourceHeader.SetData(FlowInitial,m_id,m_id_flow_initial);
		    Ptr<Packet> p = Create<Packet> (SIZE_PKT_INITIAL_FLOW);
		    p->AddHeader (sourceHeader);
		    m_SendSocket->Send(p);
			m_id_flow_initial++;
			m_num_flow_setup++;
		}
		else if (pktType == FlowRemove)
		{
			NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Switch " << m_id << " Send a flow remove packet!");
		    MyHeader sourceHeader;
		    sourceHeader.SetData(FlowRemove,m_id,0);
		    Ptr<Packet> p = Create<Packet> (SIZE_PKT_FLOW_MOD);
		    p->AddHeader (sourceHeader);
		    m_SendSocket->Send(p);
			m_num_flow_remove++;
		}
		else
		{
			NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Switch " << m_id << " Send a wrong packet!");
			//exit (EXIT_FAILURE);
		}
	}

	void SendStatsRes(uint32_t PacketID)
	{
		NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Switch " << m_id << " Send a status response packet!");
	    MyHeader sourceHeader;
	    sourceHeader.SetData(StatsRes,0,PacketID);
	    Ptr<Packet> p = Create<Packet> (SIZE_PKT_STATS_RES);
	    p->AddHeader (sourceHeader);
	    m_SendSocket->Send(p);
		m_num_status_report++;
	}
	/*
	 * Check whether the flow is expired, if yes, send flow removed packet to the Controller
	 */
	void CheckFlow()
	{
		m_vNumFlow.push_back(m_numFlow);
		bool isSend = false;
		for (std::list<Time>::iterator it = m_vInsertTime.begin(); it != m_vInsertTime.end(); ++it)
		{
			if ((*it) + Seconds(timeout) < Simulator::Now())
			{
				m_numFlow--;
				*it = Seconds(0);
				isSend = true;
			}
		}
		if (isSend)
		{
			SendPacket(FlowRemove); // send the flow remove packet to the Controller
		}
		m_vInsertTime.remove(Seconds(0));
		Simulator::Schedule(MilliSeconds(pollInterval),&Switch::CheckFlow, this);
	}

	/*
	 * \brief receive the packet (flow mod, status request) from the Controller
	 */
	void ReceivePacket (Ptr<Socket> socket)
	{
	  Ptr<Packet> packet = socket->Recv ();
	  // you can now remove the header from the packet:
	  MyHeader destinationHeader;
	  packet->RemoveHeader (destinationHeader);

	  // we use the size of the packet to identify the type of the packet
	  if (destinationHeader.GetPacketType() == FlowMod)
	  {
		  NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Switch " << m_id << " Received a flow mod packet!");
		  if (destinationHeader.GetSenderID() == m_id && !m_mFlowInitial.empty())
		  {
			  std::map<uint32_t,double>::iterator it = m_mFlowInitial.find(destinationHeader.GetPacketID());
			  if (it != m_mFlowInitial.end())
			  {
				  m_vDelay.push_back(Simulator::Now().GetSeconds() - it->second);
				  m_mFlowInitial.erase(it);
			  }
		  }
		  m_num_flow_mod++;
		  //once receive the flow mod, install a new flow in the switch
		  m_numFlow++;
		  m_vInsertTime.push_back(Simulator::Now());
	  }
	  else if (destinationHeader.GetPacketType() == StatsReq)
	  {
		  //once receive the status request, after a delay, repay status response packet
		  NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Switch " << m_id << " Received a status request packet!");
		  m_num_status_request++;
		  Simulator::Schedule(MicroSeconds(K*m_numFlow),&Switch::SendStatsRes,this, destinationHeader.GetPacketID());
	  }
	  else
	  {
		  NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Switch " << m_id << " Received a wrong packet!");
		 // exit (EXIT_FAILURE);
	  }
	}

	void EstablishControllerConnection (Ptr<Socket> socket,Ipv4Address localAddress,
		    Ipv4Address remoteAddress)
	{
		m_SendSocket = socket;
      Address a = InetSocketAddress (localAddress, OFCONTROLLERPORT);
	  Address b = InetSocketAddress (remoteAddress, OFCONTROLLERPORT);
	  NS_ASSERT (m_SendSocket->Bind (a) == 0);
	  m_SendSocket->SetConnectCallback (
		MakeCallback (&Switch::ConnectionSucceeded, this),
		MakeCallback (&Switch::ConnectionFailed, this));
	  NS_ASSERT (m_SendSocket->Connect (b) == 0);
	}

	void ConnectionSucceeded (Ptr<Socket> socket)
	{
	    NS_LOG_FUNCTION (this << socket);
	    // Set the receive callback and create the SdnConnection
	    socket->SetRecvCallback (MakeCallback (&Switch::ReceivePacket,this));
	    NS_LOG_INFO (Simulator::Now().GetSeconds() << " Connection succeeded");
	}

	void ConnectionFailed (Ptr<Socket> socket)
	{
	    NS_LOG_FUNCTION (this << socket);
	    NS_LOG_INFO (Simulator::Now().GetSeconds() << " Connection failed");
	}
};

class Controller
{
public:
	uint32_t m_V; // the number of switches controlled by the Controller
	uint32_t m_diameter; // the diameter of the network controlled by the Controller
	std::vector<Ptr<Socket> > m_vSendSocket; // the vector of the send sockets from the Controller to the switches
	//std::vector<Ptr<Socket> > m_vReceiveSocket; // the vector of the receive sockets
	Ptr<Socket> m_ReceiveSocket;
	std::vector<double> m_vDelay; // the vector of status response delay for each flow
	//double m_lastSend; // the time when the last status request is sent
	Time m_ReqInterval; // the period for the statistic pulling
	uint32_t m_num_flow_setup; // the number of flow setup packets received
	uint32_t m_num_flow_remove; // the number of flow remove packets received
	uint32_t m_num_status_report; //the number of status report packets received
	uint32_t m_num_flow_mod; // the number of flow modification packets sent
	uint32_t m_num_status_request; // the number of status request packets sent

	//std::queue<uint32_t> m_QSentStatusReq; // the queue of the status request packet sent
	//std::queue<double> m_QLastSend; // the queue of the time the status request packet is sent
	std::map<uint32_t, double> m_mStatusReq;
	uint32_t m_id_status_req; // the id of the flow initial packet

public:
	Controller(uint32_t V, uint32_t diameter, Time ReqInterval): m_V(V), m_diameter(diameter),
	m_ReqInterval(ReqInterval),m_num_flow_setup (0), m_num_flow_remove (0),
	m_num_status_report(0), m_num_flow_mod(0), m_num_status_request(0), m_id_status_req(1){

	}
	/*
	 * generate the time for response a initial flow setup packet
	 */
	Time FlowSetUpDelay()
	{
		return NanoSeconds(C*m_V*m_V);
	}

	/*
	 * send flow mod message to the switch
	 */
	void SendFlowMod(Ptr<Socket> socket, uint16_t sendID, uint32_t id_flow_mod)
	{
	  NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Controller sends a flow mod packet!");
	  m_num_flow_mod++;
	  MyHeader sourceHeader;
	  sourceHeader.SetData(FlowMod,sendID,id_flow_mod);
	  Ptr<Packet> p = Create<Packet> (SIZE_PKT_FLOW_MOD);
	  p->AddHeader (sourceHeader);
	  socket->Send(p);
	}

	/*
	 * Periodically send the status request packet to the switch
	 */
	void SendReq()
	{
		for (std::vector<Ptr<Socket> >::iterator it = m_vSendSocket.begin();
				it != m_vSendSocket.end(); ++it)
		{
			m_mStatusReq.insert(std::pair<uint32_t,double>(m_id_status_req,Simulator::Now().GetSeconds()));
		    MyHeader sourceHeader;
		    sourceHeader.SetData(StatsReq,0,m_id_status_req);
		    Ptr<Packet> p = Create<Packet> (SIZE_PKT_STATS_REQ);
		    p->AddHeader (sourceHeader);
			(*it)->Send(p);
			NS_LOG_INFO( Simulator::Now().GetSeconds() << " Controller sends a status request packet");
			m_num_status_request++;
			m_id_status_req++;
		}
		Simulator::Schedule(m_ReqInterval,&Controller::SendReq, this);
	}



	/*
	 * receive the packet (initial flow setup packet, flow remove, status response) from the switch
	 */
	void HandleRead(Ptr<Socket> socket)
	{
		Ptr<Packet> packet = socket->Recv ();
		MyHeader destinationHeader;
		packet->RemoveHeader (destinationHeader);
		if (destinationHeader.GetPacketType() == FlowInitial)
		{
			// receive the initial flow setup packet, comupute the shortest path, send the flow mod packet with a delay
			NS_LOG_INFO( Simulator::Now().GetSeconds() << " Controller receives a initial flow setup packet");
			m_num_flow_setup++;
			for (std::vector<Ptr<Socket> >::iterator it = m_vSendSocket.begin();
					it != m_vSendSocket.end(); ++it)
			{
				if (socket == *it)
				{
					Simulator::Schedule(FlowSetUpDelay(),&Controller::SendFlowMod,this,*it,destinationHeader.GetSenderID(),destinationHeader.GetPacketID());
				}
				else if (std::rand()%101/100.0 < (m_diameter - 1.0)/(m_V - 1.0) )
				{
					Simulator::Schedule(FlowSetUpDelay(),&Controller::SendFlowMod,this,*it,destinationHeader.GetSenderID(),destinationHeader.GetPacketID());
				}
			}
		}
		else if (destinationHeader.GetPacketType() == FlowRemove)
		{
			// receive the flow remove packet from the switch, do nothing
			NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Controller Receives a flow remove packet!");
			m_num_flow_remove++;
		}
		else if (destinationHeader.GetPacketType() == StatsRes)
		{
			if (destinationHeader.GetSenderID() == 0 && !m_mStatusReq.empty())
			{
				  std::map<uint32_t,double>::iterator it = m_mStatusReq.find(destinationHeader.GetPacketID());
				  if (it != m_mStatusReq.end())
				  {
					  m_vDelay.push_back(Simulator::Now().GetSeconds() - it->second);
					  m_mStatusReq.erase(it);
				  }
			}

			NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Controller Receives a status report packet!");
			m_num_status_report++;
		}
		else
		{
			NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Controller receives a wrong packet");
			//exit (EXIT_FAILURE);
		}
	}

	void StartController(Ptr<Socket> socket)
	{
		m_ReceiveSocket = socket;
	  Address a = InetSocketAddress (Ipv4Address::GetAny (), OFCONTROLLERPORT);
	  NS_ASSERT (m_ReceiveSocket->Bind (a) == 0);

	  m_ReceiveSocket->SetAcceptCallback (
		MakeNullCallback<bool, Ptr<Socket>, const Address &> (),
		MakeCallback (&Controller::HandleAccept, this));
	  m_ReceiveSocket->SetCloseCallbacks (
		MakeCallback (&Controller::HandlePeerClose, this),
		MakeCallback (&Controller::HandlePeerError, this));
	  NS_ASSERT (socket->Listen() == 0);
	}

	void HandlePeerClose (Ptr<Socket> socket)
	{
	  NS_LOG_FUNCTION (this << socket);
	  NS_LOG_INFO (Simulator::Now().GetSeconds() << " Peer closed");
	}

	void HandlePeerError (Ptr<Socket> socket)
	{
	  NS_LOG_FUNCTION (this << socket);
	  NS_LOG_INFO (Simulator::Now().GetSeconds() << " Peer error " << socket->GetErrno ());
	}

	void HandleAccept (Ptr<Socket> s, const Address& from)
	{
	  NS_LOG_FUNCTION (this << s << from);
	  if (InetSocketAddress::IsMatchingType (from))
	    {
		 // m_vSendSocket.push_back(s);
	     // m_vSendSocket.back()->SetRecvCallback (MakeCallback (&Controller::HandleRead, this));
		  s->SetRecvCallback (MakeCallback (&Controller::HandleRead, this));
		  m_vSendSocket.push_back(s);
	    }
	}
};

double expon(double x)
{
  double z;                     // Uniform random number (0 < z < 1)
  double exp_value;             // Computed exponential value to be returned

  // Pull a uniform random number (0 < z < 1)
  do
  {
    z = std::rand()%10001/10000.0;
  }
  while ((z == 0) || (z == 1));

  // Compute exponential random variable using inversion method
  exp_value = -x * log(z);

  return(exp_value);
}

void Statistics(std::vector<double> &data, double &average, double &std)
{
	double sum = 0;
	int Num = 0;
	//std::cout << std::endl;
	for (std::vector<double>::iterator it = data.begin(); it != data.end(); ++it)
	{
		//std::cout << *it << " ";
		sum += *it;
		Num++;
	}
	average = sum / Num;
	sum = 0;
	for (std::vector<double>::iterator it = data.begin(); it != data.end(); ++it)
	{
		sum += (*it - average)*(*it - average);
	}
	std = std::sqrt(sum / Num);
	//std::cout << std::endl;
}

const char* NumConvertString(std::string type)
{
	std::ostringstream convert;
	std::string file_string = "simulationResult/scalability/p2pLocal_" + type + "_Diameter=";
	convert << Diameter;
	file_string += convert.str() + "_ControllerDiameter=";
	convert.str("");
	convert << Diameter_controller;
	file_string += convert.str();
	file_string += "_ControlDataRate=" + ControlDataRate;
	file_string += "_numControllers=";
	convert.str("");
	convert << numControllers;
	file_string += convert.str() + ".txt";
	return file_string.c_str();
}

int
main (int argc, char *argv[])
{

  CommandLine cmd;
  cmd.AddValue ("SIZE_PKT_STATS_REQ", "Packet size of status request", SIZE_PKT_STATS_REQ);
  cmd.AddValue ("SIZE_PKT_STATS_RES", "Packet size of status response", SIZE_PKT_STATS_RES);
  cmd.AddValue ("C", "Coefficient for computing the response time of flow setup", C);
  cmd.AddValue ("K", "Coefficient for computing the statistics pulling time", K);
  cmd.AddValue ("timeout", "Hard time out for flow entries", timeout);
  cmd.AddValue ("pullInterval", "Interval between statistic pulling", pullInterval);
  cmd.AddValue ("numVertices", "Number of vertices (switches) in the network", numVertices);
  cmd.AddValue ("Diameter", "Diameter of the network", Diameter);
  cmd.AddValue ("N_flow", "Number of flows generated by each switch", N_flow);
  cmd.AddValue("numControllers", "number of controllers in the network", numControllers);
  cmd.AddValue("Diameter_controller", "Average number of controllers a flow going through", Diameter_controller);
  cmd.AddValue ("ControlDataRate", "Data rate of the control channel", ControlDataRate);
  cmd.AddValue("ControlDelay", "Delay of the control channel", ControlDelay);
  cmd.AddValue("aver_interval","Average interval between flow arriving", aver_interval);
  cmd.Parse (argc, argv);

  double delayAS = Diameter; // the time for a flow goes through one domain.
  //LogComponentEnable("P2PLocalSDN",LOG_LEVEL_INFO);

  std::vector<std::vector<NetDeviceContainer> > Switch2ControllerNetDevice (numControllers, std::vector<NetDeviceContainer>(numVertices));
  std::vector<std::vector<Ipv4InterfaceContainer> > Switch2ControllerIpv4Interface (numControllers, std::vector<Ipv4InterfaceContainer>(numVertices));
  std::vector<std::vector<Switch> > v_SwitchClass(numControllers,std::vector<Switch>(numVertices));
  std::vector<Controller> v_ControllerClass(numControllers, Controller(numVertices, Diameter, Seconds(pullInterval)));

  // create switches
  NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Create switch nodes.");
  std::vector<NodeContainer> v_switch(numControllers);
  for (int i = 0; i < numControllers; ++i)
  {
	  v_switch[i].Create(numVertices);
  }
  for (int i = 0; i < numControllers; ++i)
  {
	  for (int j = 0; j < numVertices; ++j)
	  {
		  v_SwitchClass[i][j].m_id = i*numVertices + j;
	  }
  }

  // create Controller
  NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Create Controller");
  NodeContainer v_Controller;
  v_Controller.Create(numControllers);


  // install stack protocol
  InternetStackHelper internet;
  for (std::vector<NodeContainer>::iterator it = v_switch.begin(); it != v_switch.end(); ++it)
  {
	  internet.Install (*it);
  }
  internet.Install(v_Controller);

  NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Create channels.");
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute("DataRate",StringValue(ControlDataRate));
  p2p.SetChannelAttribute("Delay",StringValue(ControlDelay));

  // connect the switch with the Controller
  for (int j = 0; j < numControllers; ++j)
  {
	  for (int i = 0; i < numVertices; ++i)
	  {
		  Switch2ControllerNetDevice[j][i] = p2p.Install(v_switch[j].Get(i),v_Controller.Get(j));
	  }
  }


  // assign the ip address
  std::ostringstream oss;
  Ipv4AddressHelper ipv4;
  Ipv4InterfaceContainer ipv4_address_container;
  for (int j = 0; j < numControllers; ++j)
  {
	  for (int i = 0; i < numVertices; ++i)
	  {
		  oss.str ("");
		  oss << "10." << j << "." << i << ".0";
		  ipv4.SetBase (oss.str ().c_str (), "255.255.255.0");
		  Switch2ControllerIpv4Interface[j][i] = ipv4.Assign(Switch2ControllerNetDevice[j][i]);
	  }
  }

  // start the Controller
  TypeId tid = TypeId::LookupByName ("ns3::TcpSocketFactory");
  for (int i = 0; i < numControllers; ++i)
  {
	  Ptr<Socket> socket = Socket::CreateSocket (v_Controller.Get(i), tid);
	  v_ControllerClass[i].StartController(socket);
  }


  // establish the controller-switch connection
  for (int j = 0; j < numControllers; ++j)
  {
	  for (int i = 0; i < numVertices; ++i)
	  {
		  Ptr<Socket> socket = Socket::CreateSocket (v_switch[j].Get(i), tid);
		  Ipv4Address t_local =  Switch2ControllerIpv4Interface[j][i].GetAddress(0);
		  Ipv4Address t_remote = Switch2ControllerIpv4Interface[j][i].GetAddress(1);
		  v_SwitchClass[j][i].EstablishControllerConnection(socket, t_local, t_remote);
	  }
  }

  // Schedule sending initial flow setup packet
  for (int k = 0; k < numControllers; ++k)
  {
	  for (int i = 0; i < numVertices; ++i)
	  {
		  double scheduletime = 500 + i * 10;
		  for (uint32_t j = 0; j < N_flow; ++j)
		  {
			  Simulator::ScheduleWithContext(v_SwitchClass[k][i].m_SendSocket->GetNode()->GetId(),
					  MilliSeconds(scheduletime),&Switch::SendPacket, &v_SwitchClass[k][i],FlowInitial);
			  double delay = delayAS;
			  for (int l = 0; l < numControllers; ++l)
			  {
				  if (l != k && std::rand()%101/100.0 < (Diameter_controller-1.0)/(numControllers - 1.0))
				  {
					  // the flow will go through the controller
					  int source_switch = std::rand()%numVertices;
					  Simulator::ScheduleWithContext(v_SwitchClass[l][source_switch].m_SendSocket->GetNode()->GetId(),
							  MilliSeconds(scheduletime + delay),&Switch::SendPacket, &v_SwitchClass[l][source_switch],FlowInitial);
					  delay += delayAS;
				  }
			  }
			  scheduletime += expon(aver_interval);
			  if (scheduletime > simulationTime * 1000)
			  {
				  break;
			  }
		  }
	  }
  }

  // schedule sending the status request packet
  for (int i = 0; i < numControllers; ++i)
  {
	  Simulator::ScheduleWithContext(v_Controller.Get(i)->GetId(),Seconds(1.0), &Controller::SendReq, &v_ControllerClass[i]);
  }


  // schedule checking whether the flow is time out
  for (int j = 0; j < numControllers; ++j)
  {
	  for (int i = 0; i < numVertices; ++i)
	  {
		  Simulator::ScheduleWithContext(v_SwitchClass[j][i].m_SendSocket->GetNode()->GetId(),
				  Seconds(timeout-1),&Switch::CheckFlow, &v_SwitchClass[j][i]);
	  }
  }
  NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Run Simulation.");
  Simulator::Stop(Seconds(simulationTime));
  Simulator::Run ();
  Simulator::Destroy ();

  double average, std;

  std::ofstream flow_file;
  flow_file.open(NumConvertString("flowNum"));
  std::ofstream delay_file;
  delay_file.open(NumConvertString("StatusDelay"));
  std::ofstream FlowSetup_file;
  FlowSetup_file.open(NumConvertString("FlowSetupDelay"));
  for (int j = 0; j < numControllers; ++j)
  {
      delay_file << "Controller " << j << "\n";
	  for (std::vector<double>::iterator it = v_ControllerClass[j].m_vDelay.begin();
			  it != v_ControllerClass[j].m_vDelay.end(); ++it)
	  {
		  delay_file << *it << " ";
	  }
	  delay_file.close();

	  std::cout << "Controller " << j << std::endl;
	  std::cout << v_ControllerClass[j].m_num_flow_mod * SIZE_PKT_FLOW_MOD << " "
			  << v_ControllerClass[j].m_num_status_request * SIZE_PKT_STATS_REQ << " "
			  << v_ControllerClass[j].m_num_flow_remove * SIZE_PKT_FLOW_REMOVE << " "
			  << v_ControllerClass[j].m_num_flow_setup * SIZE_PKT_INITIAL_FLOW << " "
			  << v_ControllerClass[j].m_num_status_report * SIZE_PKT_STATS_RES << " ";
	  Statistics(v_ControllerClass[j].m_vDelay,average, std);
	  std::cout << average << " " << std << std::endl;

	  flow_file << "Controller " << j << "\n";
	  int switch_sum_packet[5] = {};
	  for (int i = 0; i < numVertices; ++i)
	  {
		  switch_sum_packet[0] += v_SwitchClass[j][i].m_num_flow_mod;
		  switch_sum_packet[1] += v_SwitchClass[j][i].m_num_status_request;
		  switch_sum_packet[2] += v_SwitchClass[j][i].m_num_flow_remove;
		  switch_sum_packet[3] += v_SwitchClass[j][i].m_num_flow_setup;
		  switch_sum_packet[4] += v_SwitchClass[j][i].m_num_status_report;
		 // std::cout << "Switch " << i << std::endl;
		  std::cout << v_SwitchClass[j][i].m_num_flow_mod * SIZE_PKT_FLOW_MOD << " "
				  << v_SwitchClass[j][i].m_num_status_request * SIZE_PKT_STATS_REQ << " "
				  << v_SwitchClass[j][i].m_num_flow_remove * SIZE_PKT_FLOW_REMOVE << " "
				  << v_SwitchClass[j][i].m_num_flow_setup * SIZE_PKT_INITIAL_FLOW << " "
				  << v_SwitchClass[j][i].m_num_status_report * SIZE_PKT_STATS_RES << " "
				  << v_SwitchClass[j][i].m_numFlow << " ";
		  Statistics(v_SwitchClass[j][i].m_vDelay,average, std);
		  std::cout << average << " " << std << std::endl;

		  for (std::vector<uint32_t>::iterator it = v_SwitchClass[j][i].m_vNumFlow.begin();
				  it != v_SwitchClass[j][i].m_vNumFlow.end(); ++it)
		  {
			  flow_file << *it << " ";
		  }
		  flow_file << "\n";

		  FlowSetup_file << "Controller " << j << " Switch " << i << "\n";
		  for (std::vector<double>::iterator it = v_SwitchClass[j][i].m_vDelay.begin();
				  it != v_SwitchClass[j][i].m_vDelay.end();++it)
		  {
			  FlowSetup_file << *it << " ";
		  }
		  FlowSetup_file << "\n";

	  }
	  std::cout << "packet loss rate" << std::endl;
	  std::cout << switch_sum_packet[0]/(v_ControllerClass[j].m_num_flow_mod + 0.0) << " "
			  << switch_sum_packet[1]/(v_ControllerClass[j].m_num_status_request + 0.0) << " "
			  << switch_sum_packet[2]/(v_ControllerClass[j].m_num_flow_remove + 0.0)<< " "
			  << switch_sum_packet[3]/(v_ControllerClass[j].m_num_flow_setup + 0.0) << " "
			  << switch_sum_packet[4]/(v_ControllerClass[j].m_num_status_report + 0.0)<< std::endl;
	  std::cout << "*********************************************" << std::endl;
  }
  flow_file.close();
  delay_file.close();

  std::cout << "In summary" << std::endl;
  // compute the average and stdev of the flow setup delay
  std::vector<double> v_setup_delay;
  for (int j = 0; j < numControllers; ++j)
  {
	  for (int i = 0; i < numVertices; ++i)
	  {
		  v_setup_delay.insert(v_setup_delay.end(),v_SwitchClass[j][i].m_vDelay.begin(), v_SwitchClass[j][i].m_vDelay.end());
	  }
  }
  Statistics(v_setup_delay,average, std);
  std::cout << average << " " << std << " ";

  // compute the average and stdev of flow setup delay for the local controller
  v_setup_delay.clear();
  for (int j = 0; j < numControllers; ++j)
  {
	  v_setup_delay.insert(v_setup_delay.end(),v_ControllerClass[j].m_vDelay.begin(), v_ControllerClass[j].m_vDelay.end());
  }
  Statistics(v_setup_delay,average, std);
  std::cout << average << " " << std << " ";

  // compute the average and stdev of the number of flows
  std::vector<double> v_flow_num;
  for (int j = 0; j < numControllers; ++j)
  {
	  for (int i = 0; i < numVertices; ++i)
	  {
		  v_flow_num.push_back(double(v_SwitchClass[j][i].m_numFlow));
	  }
  }
  Statistics(v_flow_num,average,std);
  std::cout << average << " " << std << std::endl;
  std::cout << "*********************************************" << std::endl;

  NS_LOG_INFO ( Simulator::Now().GetSeconds() << " Done.");
}