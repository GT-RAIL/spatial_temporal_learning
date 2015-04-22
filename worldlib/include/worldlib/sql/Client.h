/*!
 * \file Client.h
 * \brief The main MySQL client connection.
 *
 * The worldlib SQL client can communicate with a MySQL database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_SQL_CLIENT_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_SQL_CLIENT_H_

// MySQL
#include <mysql/mysql.h>

// C++ Standard Library
#include <stdint.h>
#include <string>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace sql
{

/*!
 * \class Client
 * \brief The main MySQL client connection.
 *
 * The worldlib SQL client can communicate with a MySQL database.
 */
class Client
{
public:
  /*! The default MySQL port. */
  static const unsigned int DEFAULT_PORT = 3306;

  /*!
   * \brief Create a new Client.
   *
   * Creates a new Client by copying the values from the given Client. A new connection is made if one exists.
   *
   * \param client The Client to copy.
   */
  Client(const Client &client);

  /*!
   * \brief Create a new Client.
   *
   * Creates a new Client with the given connection information. A connection is not made by default.
   *
   * \param host The host of the database.
   * \param port The host port of the database.
   * \param user The user of the database.
   * \param password The password for the user of the database.
   * \param database The database name.
   */
  Client(const std::string &host, const uint16_t port, const std::string &user, const std::string &password,
      const std::string &database);

  /*!
   * \brief Cleans up a Client.
   *
   * Cleans up any connections used by the Client.
   */
  virtual ~Client();

  /*!
   * \brief Port value accessor.
   *
   * Get the port value of this Client.
   *
   * \return The port value.
   */
  uint16_t getPort() const;

  /*!
   * \brief Host value accessor.
   *
   * Get the host value of this Client.
   *
   * \return The host value.
   */
  const std::string &getHost() const;

  /*!
   * \brief User value accessor.
   *
   * Get the user value of this Client.
   *
   * \return The user value.
   */
  const std::string &getUser() const;

  /*!
   * \brief Password value accessor.
   *
   * Get the password value of this Client.
   *
   * \return The password value.
   */
  const std::string &getPassword() const;

  /*!
   * \brief Database value accessor.
   *
   * Get the database value of this Client.
   *
   * \return The database value.
   */
  const std::string &getDatabase() const;

  /*!
   * \brief Check if there is a connection to the database.
   *
   * A boolean check to see if a connection exists to the database.
   *
   * \return True if a connection has been made.
   */
  bool connected() const;

  /*!
   * \brief Create a connection to the database.
   *
   * Attempts to create a connection to the database. A flag is returned to indicate the success.
   *
   * \return True if a connection has been successfully made.
   */
  virtual bool connect();

  /*!
   * \brief Closes a connection to the database.
   *
   * Attempts to close a connection to the database. No effect is seen if there is no current connection.
   */
  virtual void disconnect();

protected:
  /*!
   * \brief Execute a SQL query in the database.
   *
   * Attempt to run a SQL query in the database and return the result. If no connection is made or an error occurred,
   * NULL is returned. It is the job of the caller to cleanup and returned pointer via mysql_free_result.
   *
   * \param query The SQL query to run.
   * \return The SQL result or NULL if an error occurred.
   */
  MYSQL_RES *query(std::string query) const;

  /*!
   * \brief Print the latest SQL error to ROS_ERROR.
   *
   * Prints the latest SQL error message to the ROS error stream.
   */
  void printSqlError() const;

private:
  /*! Database connection information. */
  std::string host_, user_, password_, database_;
  /*! Database port information. */
  uint16_t port_;
  /*! Connection flag */
  bool connected_;
  /*! The main database connection client. */
  MYSQL *connection_;
};

}
}
}
}

#endif
