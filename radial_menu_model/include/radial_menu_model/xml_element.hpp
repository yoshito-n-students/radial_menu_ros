#ifndef RADIAL_MENU_MODEL_XML_ELEMENT_HPP
#define RADIAL_MENU_MODEL_XML_ELEMENT_HPP

#include <sstream>
#include <string>
#include <vector>

#include <ros/console.h>

#include <boost/optional.hpp>
#include <boost/property_tree/exceptions.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/shared_ptr.hpp>

namespace radial_menu_model {

class XmlElement;
typedef boost::shared_ptr< XmlElement > XmlElementPtr;
typedef boost::shared_ptr< const XmlElement > XmlElementConstPtr;

class XmlElement {
protected:
  XmlElement(const boost::shared_ptr< const boost::property_tree::ptree > &xml,
             const boost::property_tree::ptree::value_type *const elm)
      : xml_(xml), elm_(elm) {}

public:
  virtual ~XmlElement() {}

  // element name
  const std::string name() const { return elm_->first; }

  // get an attribute like ros::param::param()
  template < typename T > T attribute(const std::string &attr, const T &default_val) const {
    const boost::optional< T > val(elm_->second.get_optional< T >("<xmlattr>." + attr));
    return val ? *val : default_val;
  }

  // get an attribute like ros::param::get()
  template < typename T > bool getAttribute(const std::string &attr, T *const val) const {
    const boost::optional< T > opt_val(elm_->second.get_optional< T >("<xmlattr>." + attr));
    if (opt_val) {
      *val = *opt_val;
      return true;
    }
    return false;
  }

  std::size_t numChildElements() const {
    std::size_t n(0);
    for (const boost::property_tree::ptree::value_type &child_elm : elm_->second) {
      if (child_elm.first != "<xmlattr>") {
        ++n;
      }
    }
    return n;
  }

  std::vector< XmlElementConstPtr > childElements() const {
    std::vector< XmlElementConstPtr > elms;
    for (const boost::property_tree::ptree::value_type &child_elm : elm_->second) {
      if (child_elm.first != "<xmlattr>") {
        elms.push_back(XmlElementConstPtr(new XmlElement(xml_, &child_elm)));
      }
    }
    return elms;
  }

  static XmlElementConstPtr fromString(const std::string &str) {
    namespace bpt = boost::property_tree;

    // parse the given string as a xml
    const boost::shared_ptr< bpt::ptree > xml(new bpt::ptree());
    try {
      std::istringstream iss(str);
      bpt::read_xml(iss, *xml, bpt::xml_parser::no_comments);
    } catch (const bpt::ptree_error &ex) {
      ROS_ERROR_STREAM("XmlElement::fromString(): " << ex.what());
      return XmlElementConstPtr();
    }

    // assert the xml has the unique root element
    if (xml->empty()) {
      ROS_ERROR("XmlElement::fromString(): No root element in xml");
      return XmlElementConstPtr();
    } else if (xml->size() >= 2) {
      ROS_ERROR("XmlElement::fromString(): Multiple root elements in xml");
      return XmlElementConstPtr();
    }

    // return the root element
    return XmlElementConstPtr(new XmlElement(xml, &xml->front()));
  }

protected:
  const boost::shared_ptr< const boost::property_tree::ptree > xml_;
  const boost::property_tree::ptree::value_type *const elm_;
};
} // namespace radial_menu_model

#endif