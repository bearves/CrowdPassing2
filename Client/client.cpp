#include <aris.h>

int sendRequest(int argc, char *argv[], const char *xmlFileName)
{
    /*��Ҫȥ���������·������չ��*/
    std::string cmdName(argv[0]);

#ifdef WIN32
    if (cmdName.rfind('\\'))
    {
        cmdName = cmdName.substr(cmdName.rfind('\\') + 1, cmdName.npos);
    }
#endif
#ifdef UNIX
    if (cmdName.rfind('/'))
    {
        cmdName = cmdName.substr(cmdName.rfind('/') + 1, cmdName.npos);
    }
#endif

    if (cmdName.rfind('.'))
    {
        cmdName = cmdName.substr(0, cmdName.rfind('.'));
    }

    /*�����������в���*/
    for (int i = 1; i < argc; ++i)
    {
        cmdName = cmdName + " " + argv[i];
    }



    /*����msg��������Ҫ��copy������ƣ�Ȼ������copy��������*/
    Aris::Core::Msg msg;
    msg.copy(cmdName.c_str());



    /*���Ӳ�����msg*/
    Aris::Core::XmlDocument doc;

    if (doc.LoadFile(xmlFileName) != 0)
        throw std::logic_error("failed to read configuration xml file");

    std::string ip = doc.RootElement()->FirstChildElement("Server")->Attribute("ip");
    std::string port = doc.RootElement()->FirstChildElement("Server")->Attribute("port");

    Aris::Core::Socket conn;

    while (true)
    {
        try
        {
            conn.connect(ip.c_str(), port.c_str());
            break;
        }
        catch (std::exception &)
        {
            std::cout << "failed to connect server, will retry in 1 second" << std::endl;
            Aris::Core::msSleep(1000);
        }

    }

    Aris::Core::Msg ret = conn.sendRequest(msg);

    /*������*/
    if (ret.size() > 0)
    {
        std::cout << "cmd has fault, please regard to following information:" << std::endl;
        std::cout << "    " << ret.data() << std::endl;
    }
    else
    {
        std::cout << "send command successful" << std::endl;
    }

    return 0;
}

int main(int argc, char *argv[])
{
    if (argc <= 1)throw std::runtime_error("please input the cmd name");

#ifdef UNIX
    sendRequest(argc - 1, argv + 1, "/usr/Robots2/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif
#ifdef WIN32
    sendRequest(argc - 1, argv + 1, "C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif

    return 0;
}
