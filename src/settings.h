/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \brief Simple application settings utility built on boost
 *  property tree.
 *
 */

#ifndef QUANERGY_CLIENT_ROS_SETTINGS_H
#define QUANERGY_CLIENT_ROS_SETTINGS_H

// for debugging
#include <iostream>

#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>


namespace quanergy
{

  class Settings
  {
  public:

    void loadXML(std::string const & file_name);

    template <typename T>
    T get(std::string const & setting_name) const;

    template <typename T>
    T get(std::string const & setting_name, T const & default_value) const;

    template <typename T>
    void set(std::string const & setting_name, T const & value);

  private:
    boost::property_tree::ptree ptree;

  };



  inline void Settings::loadXML(std::string const & file_name)
  {
    boost::property_tree::xml_parser::read_xml(file_name, ptree);
  }

  template <typename T>
  inline T Settings::get(std::string const & setting_name) const
  {
    const T& ret = ptree.get<T>(std::string("Settings.") + setting_name);
    return ret;
  }

  template <typename T>
  inline T Settings::get(std::string const & setting_name, T const & default_value) const
  {
    const T& ret = ptree.get<T>(std::string("Settings.") + setting_name, default_value);
    return ret;
  }

  template <typename T>
  inline void Settings::set(std::string const & setting_name, T const & value)
  {
    ptree.put<T>(std::string("Settings.") + setting_name, value);
  }


} // namespace quanergy



#endif
