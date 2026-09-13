const path=require("path");
const root=process.env.AGV_ROOT||path.resolve(__dirname,"..");
const baseURL=(process.env.ROS_WEB_BASE_URL||"http://127.0.0.1:5000").replace(/\/+$/,"");
module.exports={testDir:path.join(root,".playwright"),timeout:30000,use:{headless:true,baseURL},workers:1};
