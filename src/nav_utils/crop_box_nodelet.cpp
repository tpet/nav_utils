#include <thread>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter_indices.h>

#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

namespace nav_utils {

class CropBoxPointCloud2 : public pcl::CropBox<pcl::PCLPointCloud2>
{
  void applyFilter(pcl::PCLPointCloud2 &output) override;
};

void CropBoxPointCloud2::applyFilter(pcl::PCLPointCloud2 &output)
{
  if (!this->keep_organized_) {
    pcl::CropBox<pcl::PCLPointCloud2>::applyFilter(output);
  } else {
    // this is basically a copy-paste from
    // https://github.com/PointCloudLibrary/pcl/blob/pcl-1.10.1/filters/src/crop_box.cpp
    // BSD license, Copyright (c) 2015, Google, Inc.
    // minor modifications Martin Pecka, 2020
    const auto temp = extract_removed_indices_;
    extract_removed_indices_ = true;
    std::vector<int> indices;
    pcl::CropBox<pcl::PCLPointCloud2>::applyFilter(indices);
    extract_removed_indices_ = temp;
    PCL_DEBUG("[pcl::CropBox<pcl::PCLPointCloud2>::applyFilter] Removing %lu points of "
              "%lu points.\n", removed_indices_->size(), input_->height * input_->width);

    output = *input_;

    // Get x, y, z fields. We should not just assume that they are the first fields of each point
    std::vector<decltype(pcl::PCLPointField::offset)> offsets;
    for (const pcl::PCLPointField& field : input_->fields)
    {
      if (field.name == "x" || field.name == "y" || field.name == "z")
        offsets.push_back(field.offset);
    }
    PCL_DEBUG("[pcl::CropBox<pcl::PCLPointCloud2>::applyFilter] Found %lu fields called "
              "'x', 'y', or 'z'.\n", offsets.size());

    // For every "removed" point, set the x, y, z fields to user_filter_value_
    for (const auto ri : *removed_indices_) // ri = removed index
    {
      auto pt_data = reinterpret_cast<std::uint8_t*>(&output.data[ri * output.point_step]);
      for (const auto &offset : offsets)
      {
        memcpy(pt_data + offset, &user_filter_value_, sizeof(float));
      }
    }
    if (!std::isfinite(user_filter_value_))
    {
      PCL_DEBUG("[pcl::CropBox<pcl::PCLPointCloud2>::applyFilter] user_filter_value_ is %f, which "
                "is not finite, so the is_dense field of the output will be set to false.\n",
                user_filter_value_);
      output.is_dense = false;
    }
  }
}


class CropBoxNodelet: public nodelet::Nodelet {
public:
    void onInit() override
    {
      box.setMin({
        this->getPrivateNodeHandle().param("min_x", -1.0),
        this->getPrivateNodeHandle().param("min_y", -1.0),
        this->getPrivateNodeHandle().param("min_z", -1.0),
        0
      });
      box.setMax({
        this->getPrivateNodeHandle().param("max_x", 1.0),
        this->getPrivateNodeHandle().param("max_y", 1.0),
        this->getPrivateNodeHandle().param("max_z", 1.0),
        0
      });
      box.setKeepOrganized(this->getPrivateNodeHandle().param("keep_organized", true));
      box.setNegative(this->getPrivateNodeHandle().param("negative", false));
      pub_output_ = this->getPrivateNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 10);
      sub_input_ = this->getPrivateNodeHandle().subscribe<sensor_msgs::PointCloud2>
        ("input", 10, &CropBoxNodelet::callback, this);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
      sensor_msgs::PointCloud2Ptr output(new sensor_msgs::PointCloud2);
      pcl::PCLPointCloud2 pclOutput;
      // Call the virtual method in the child
      pcl::PCLPointCloud2Ptr pclInput(new pcl::PCLPointCloud2());
      pcl_conversions::toPCL(*input, *pclInput);
      box.setInputCloud(pclInput);
      box.filter(pclOutput);
      pcl_conversions::fromPCL(pclOutput, *output);

      // Copy timestamp to keep it
      output->header.stamp = input->header.stamp;

      // Publish a boost shared ptr
      pub_output_.publish (output);
    }
  private:
  ros::Subscriber sub_input_;
  ros::Publisher pub_output_;
  CropBoxPointCloud2 box;
};

}

PLUGINLIB_EXPORT_CLASS(nav_utils::CropBoxNodelet, nodelet::Nodelet)
