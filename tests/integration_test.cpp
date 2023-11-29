#include <gtest/gtest.h>
#include <zmq.hpp>
#include <Eigen/Dense>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include <thread>
#include <future>
#include <cmath>
#include <chrono>
using namespace std::chrono_literals;

#include "projectile_parser.hpp"


const std::string programPath = "./drop";
const std::string communicationFolder = "ipc:///tmp/drop_shot";

zmq::context_t ctx;

/// Test if program show correct usage 
TEST(IntegrationTest, ProgramShowsHelp) {
    testing::internal::CaptureStdout();

    int exitCode = std::system((programPath + " --help").c_str());

    std::string output = testing::internal::GetCapturedStdout();
    EXPECT_EQ(exitCode, 0) << "--help: exited with error code";
    EXPECT_TRUE(output.find("Usage:") != std::string::npos)<< "--help: No \"Usage:\" key word";

    size_t pos = output.find("Usage:");
    if (pos != std::string::npos) {
        output = output.substr(0, pos) + "\033[1;35mUsage:\033[0m" + output.substr(pos + 6);
    }
    std::cout << output;
}

class DropTest : public testing::Test {
protected:

    void SetUp() override {
        testing::internal::CaptureStdout();
        std::promise<int> p;
        retValue = p.get_future();
        dropThread = std::thread([](std::promise<int> && p){
            p.set_value(std::system(programPath.c_str()));
            }, std::move(p));
        std::this_thread::sleep_for(100ms);
        controlSocket = zmq::socket_t(ctx, zmq::socket_type::req);
        controlSocket.set(zmq::sockopt::rcvtimeo,1000);
        controlSocket.set(zmq::sockopt::sndtimeo,1000);
        controlSocket.connect(communicationFolder + "/control");
        if(controlSocket.handle() == nullptr)
        {
            FAIL() << "control socket not connected";
        }
        stateSocket = zmq::socket_t(ctx, zmq::socket_type::sub);
        stateSocket.set(zmq::sockopt::subscribe, "");
        controlSocket.set(zmq::sockopt::rcvtimeo,100);
        controlSocket.set(zmq::sockopt::conflate,1);
        stateSocket.connect(communicationFolder + "/state");
        if(stateSocket.handle() == nullptr)
        {
            FAIL() << "state socket not connected";
        }
        testing::internal::GetCapturedStdout();
    }

    void TearDown() override 
    {
        sendControlMessage("s");
        controlSocket.close();
        stateSocket.close();
        std::future_status status = retValue.wait_for(1s);
        if(status == std::future_status::ready)
        {
            dropThread.join();
            EXPECT_EQ( retValue.get(),0) << "drop exited with error code";
        }
        else
        {
            dropThread.~thread();
            FAIL() << "drop does not stop - terminated";
        }
    }

    void sendControlMessage(std::string msg, bool should_response = true, bool should_response_ok = true)
    {
        zmq::message_t message(msg);
        auto res = controlSocket.send(message, zmq::send_flags::none);
        if(!res)
        {
            FAIL() << "drop can not send message";
        } 
        if(!should_response)
        {
            return;
        }
        zmq::message_t response;
        auto res2 = controlSocket.recv(response, zmq::recv_flags::none);
        if(!res2)
        {
            FAIL() << "drop no response";
        } 
        std::string response_str =  std::string(static_cast<char*>(response.data()), response.size());
        //std::cout << "RESPONSE: " << response_str << std::endl;
        if(!should_response_ok)
        {
            return;
        }
        if(response_str.find("ok") == std::string::npos)
        {
            FAIL() << "drop response does not contain ok";
        }
    }

    int recvState(std::string& response_str)
    {
        zmq::message_t state;
        auto res = stateSocket.recv(state, zmq::recv_flags::none);
        if(!res)
        {
            return 1;
        }
        response_str =  std::string(static_cast<char*>(state.data()), state.size());
        return 0;
    }

    auto getParsedState()
    {
        std::string state;
        recvState(state);
        return parseInput(state);
    }

    void collectSample(const int N = 10)
    {
        double last_time = -1.0;

        for (int i = 0; i < N; i++)
        {
            std::string state;
            if(recvState(state))
            {
                FAIL() << "drop fails on state recv";
            }
            //std::cout << state << std::endl;
            if (state.find("inf") != std::string::npos || state.find("nan") != std::string::npos)
            {
                std::cout << state << std::endl;
                FAIL() << "drop state contains nan or inf";
            } 

            auto [time, projectiles] = parseInput(state);

            if(time < 0.0)
            {
                FAIL() << "drop parse state error";
            }

            if(time < last_time)
            {
                FAIL() << "drop time go backwards!?";
            }
            last_time = time;
        }
    }

private:
    std::thread dropThread;
    std::future<int> retValue;
    zmq::socket_t controlSocket;
    zmq::socket_t stateSocket;
};

/// Test if program run and exit on command correctly 
TEST_F(DropTest, ProgramRunsAndExitsCorrectly) {
    SUCCEED();
}

/// Test if program run send valid state
TEST_F(DropTest, ProgramRunsAndSendState) {
    collectSample();
}

/// Test if program handle add object and remove object command correctly
TEST_F(DropTest, AddAndRemoveProjectiles) {
    collectSample();
    std::cout << "Below is deliberatly invalid." << std::endl;
    sendControlMessage("a:1.0,0.01,1.0,2.0,",true,false);
    sendControlMessage("a:1.0,0.01,1.0,2.0,3.0");
    collectSample(3);
    auto [_, projectiles] = getParsedState();
    EXPECT_EQ(projectiles.size(),1);
    std::cout << "Below is deliberatly invalid." << std::endl;
    sendControlMessage("a:1.0,0.01,1.0,2.0,0.0,0.0",true,false);
    sendControlMessage("a:1.0,0.01,3.0,4.0,5.0,6.0,7.0,8.0");
    collectSample(3);
    projectiles = getParsedState().second;
    EXPECT_EQ(projectiles.size(),2);

    if (std::find_if(projectiles.begin(), projectiles.end(),
                     [](const Projectile &x) { return x.id == 0; }) ==
            projectiles.end() ||
        std::find_if(projectiles.begin(), projectiles.end(),
                     [](const Projectile &x) { return x.id == 1; }) ==
            projectiles.end())
    {
        FAIL() << "drop invalid id";
    }
    sendControlMessage("r:2",true,false);
    sendControlMessage("r:0");
    collectSample(3);
    projectiles = getParsedState().second;
    EXPECT_EQ(projectiles.size(),1);
    EXPECT_EQ(projectiles.front().id, 1);
}

/// Test if program simulates free fall correctly
TEST_F(DropTest, CheckFreeFall) {
    constexpr double tol = 0.1;
    collectSample();
    sendControlMessage("a:1.0,0.0,10.0,20.0,30.0");
    auto [start_time, _] = getParsedState();
    auto start = std::chrono::system_clock::now();
    auto t_wall = 0;
    while(t_wall < 300)
    {
        t_wall = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();
        collectSample(1);
    }
    auto [end_time, projectiles] = getParsedState();
    auto t = end_time - start_time;
    EXPECT_NEAR(t,t_wall/1000.0, 0.01);
    EXPECT_EQ(projectiles.size(),1);
    EXPECT_EQ(projectiles.front().id, 0);
    EXPECT_NEAR(projectiles.front().position.x(), 10.0, tol);
    EXPECT_NEAR(projectiles.front().position.y(), 20.0, tol);
    EXPECT_NEAR(projectiles.front().velocity.x(), 0.0, tol);
    EXPECT_NEAR(projectiles.front().velocity.y(), 0.0, tol);

    EXPECT_NEAR(projectiles.front().position.z(), 30.0 + 0.5 * 9.805 * t * t, tol);
    EXPECT_NEAR(projectiles.front().velocity.z(), t * 9.805, 5*tol);
}

/// Test if program simulates air drag correctly
TEST_F(DropTest, LikeParachute) {
    constexpr double tol = 0.1;
    collectSample();
    sendControlMessage("a:1.0,0.5,10.0,20.0,30.0,0.0,0.0,100.0");
    auto [start_time, _] = getParsedState();
    collectSample(100);
    auto [end_time, projectiles] = getParsedState();
    EXPECT_EQ(projectiles.size(),1);
    EXPECT_EQ(projectiles.front().id, 0);
    EXPECT_NEAR(projectiles.front().position.x(), 10.0, tol);
    EXPECT_NEAR(projectiles.front().position.y(), 20.0, tol);
    EXPECT_NEAR(projectiles.front().velocity.x(), 0.0, tol);
    EXPECT_NEAR(projectiles.front().velocity.y(), 0.0, tol);

    EXPECT_GT(projectiles.front().position.z(), 30.0);
    EXPECT_LT(projectiles.front().velocity.z(), 50.0);
}

/// Test if program simulates outer forces influence correctly
TEST_F(DropTest, ForceInfluence) {
    constexpr double tol = 0.05;
    collectSample();
    sendControlMessage("a:5.0,0.0,0.0,0.0,0.0");
    sendControlMessage("a:5.0,0.0,0.0,0.0,0.0");
    auto [start_time, _] = getParsedState();
    for (int i = 0; i < 100; i++)
    {
        sendControlMessage("f:1,2.0,1.0,0.0");
        collectSample(1);
    }
    auto [end_time, projectiles] = getParsedState();
    auto t = end_time - start_time;
    EXPECT_EQ(projectiles.size(),2);
    EXPECT_EQ(projectiles[1].id, 1);
    EXPECT_NEAR(projectiles[1].position.x(), 0.5 * 2.0 / 5.0 * t * t, tol);
    EXPECT_NEAR(projectiles[1].position.y(), 0.5 * 1.0 / 5.0 * t * t, tol);
    EXPECT_NEAR(projectiles[1].velocity.x(), 2.0 / 5.0 * t, tol);
    EXPECT_NEAR(projectiles[1].velocity.y(), 1.0 / 5.0 * t, tol);
}

/// Test if program simulates solid surface collision correctly
TEST_F(DropTest, SolidSurfaceCollision) {
    constexpr double tol = 0.05;
    const auto normal = Eigen::Vector3d(1.0,1.0,1.0).normalized();
    collectSample();
    sendControlMessage("a:5.0,0.0,0.0,0.0,0.0");

    sendControlMessage("a:5.0,0.0 ,0.0,0.0,0.0,-10.0,0.0,0.0");
    collectSample(1);
    sendControlMessage("j:1,0.0,0.0,0.0,0.577,0.577,0.577");
    collectSample(1);
    auto [_, projectiles] = getParsedState();
    EXPECT_EQ(projectiles.size(),2);
    auto vel_after_collision = projectiles[1].velocity;
    auto dot = vel_after_collision.dot(normal)/vel_after_collision.norm();
    EXPECT_NEAR(dot, 0.0, tol);

    sendControlMessage("a:5.0,0.0 ,0.0,0.0,0.0,-10.0,0.0,0.0");
    collectSample(1);
    sendControlMessage("j:2,1.0,0.0,0.0,0.577,0.577,0.577");
    collectSample(1);
    projectiles = getParsedState().second;
    EXPECT_EQ(projectiles.size(),3);
    vel_after_collision = projectiles[2].velocity;
    Eigen::Vector3d vel_diff = vel_after_collision - Eigen::Vector3d(-10.0,0.0,0.0);
    dot = vel_diff.dot(normal)/vel_diff.norm();
    EXPECT_NEAR(dot, 1.0, tol);
}

/// Test if program simulates strong wind influence correctly
TEST_F(DropTest, StrongWindInfluence) {
    constexpr double tol = 0.5;
    collectSample();
    sendControlMessage("a:5.0,0.0,0.0,0.0,0.0");
    sendControlMessage("a:5.0,2.0,0.0,0.0,0.0");
    sendControlMessage("w:1,20.0,15.0,-5.0");
    auto [start_time, _] = getParsedState();
    for (int i = 0; i < 1000; i++)
    {
        collectSample(1);
    }
    auto [end_time, projectiles] = getParsedState();
    auto t = end_time - start_time;
    EXPECT_GT(t, 3.0);
    EXPECT_EQ(projectiles.size(),2);
    EXPECT_EQ(projectiles[1].id, 1);
    EXPECT_NEAR(projectiles[1].velocity.x(), 20.0, tol);
    EXPECT_NEAR(projectiles[1].velocity.y(), 15.0, tol);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}