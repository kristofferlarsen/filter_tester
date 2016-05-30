/**
 * @file /include/agilus_master_project/qnode.hpp
 *
 * @brief Original author Kristoffer Larsen. Handles all ROS communication.
 *
 * @date May 2016
 **/

#ifndef qt_filter_tester_QNODE_HPP_
#define qt_filter_tester_QNODE_HPP_

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

namespace qt_filter_tester {

class QNode : public QThread {
    Q_OBJECT
public:

    /*!
     * \brief Constructor for the QNode class.
     * \param argc Initial argument.
     * \param argv Initial argument.
     */
	QNode(int argc, char** argv );

    /*!
     * \brief Desconstructor for the QNode class.
     */
	virtual ~QNode();

    /*!
     * \brief Initiates the ROS communication.
     * \return False if ROS initial setup failed.
     */
	bool init();

    /*!
     * \brief Initiates the ROS communication
     * \param master_url IP address of the ROS master.
     * \param host_url IP address of the local host.
     * \return
     */
	bool init(const std::string &master_url, const std::string &host_url);

    /*!
     * \brief Starts the main thread. At this point, the thread is driven by ROS communication events.
     */
	void run();

Q_SIGNALS:

    /*!
     * \brief Signals the user interface to close upon a ros shutdown.
     */
    void rosShutdown();

private:
    int init_argc; //!< Initial condition.
    char** init_argv; //!< Initial condition.
};

}  // namespace qt_test

#endif /* qt_test_QNODE_HPP_ */
