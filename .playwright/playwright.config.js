const path=require("path");
const root=process.env.AGV_ROOT||path.resolve(__dirname,"..");
module.exports={testDir:path.join(root,".playwright"),timeout:30000,use:{headless:true,baseURL:"http://127.0.0.1:5000"},workers:1};
