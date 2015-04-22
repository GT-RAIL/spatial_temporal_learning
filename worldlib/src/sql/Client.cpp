/*!
 * \file Client.cpp
 * \brief The main MySQL client connection.
 *
 * The worldlib SQL client can communicate with a MySQL database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

// worldlib
#include "worldlib/sql/Client.h"

// ROS
#include <ros/ros.h>

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::sql;

Client::Client(const Client &client)
    : host_(client.getHost()), user_(client.getUser()), password_(client.getPassword()), database_(client.getDatabase())
{
  port_ = client.getPort();
  connection_ = NULL;

  // check if a connection was made
  if (client.connected())
  {
    this->connect();
  }
}

Client::Client(const string &host, const uint16_t port, const string &user, const string &password,
    const string &database) : host_(host), user_(user), password_(password), database_(database)
{
  port_ = port;
  connection_ = NULL;
  connected_ = false;
}

Client::~Client()
{
  // check for an existing connection
  this->disconnect();
}

uint16_t Client::getPort() const
{
  return port_;
}

const string &Client::getHost() const
{
  return host_;
}

const string &Client::getUser() const
{
  return user_;
}

const string &Client::getPassword() const
{
  return password_;
}

const string &Client::getDatabase() const
{
  return database_;
}

bool Client::connected() const
{
  return connection_ != NULL && connected_;
}

bool Client::connect()
{
  // check for an existing connection
  this->disconnect();

  // setup the client connection
  connection_ = mysql_init(NULL);
  connected_ = mysql_real_connect(connection_, host_.c_str(), user_.c_str(), password_.c_str(), database_.c_str(),
                                  port_, NULL, 0);
  if (!connected_)
  {
    this->printSqlError();
  }

  return this->connected();
}

void Client::disconnect()
{
  // check for an existing connection
  if (connection_ != NULL)
  {
    if (this->connected())
    {
      mysql_close(connection_);
    }
    connection_ = NULL;
    connected_ = false;
  }
}

MYSQL_RES *Client::query(string query) const
{
  if (this->connected())
  {
    if (mysql_query(connection_, query.c_str()) == 0)
    {
      // parse and get it
      return mysql_use_result(connection_);
    } else
    {
      this->printSqlError();
    }
  } else
  {
    ROS_WARN("MySQL attempted to make a query while it was not connected.");
  }

  // something went wrong
  return NULL;
}

void Client::printSqlError() const
{
  ROS_ERROR("MySQL Error: %s", mysql_error(connection_));
}
