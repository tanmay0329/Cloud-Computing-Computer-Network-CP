import {useState} from 'react'
import { useNavigate } from 'react-router-dom'
// import { io } from "socket.io-client";
import axios from "axios";
const GovMain = () => {

  const [msg, setMsg] = useState("")
  const navigate = useNavigate();
  const [notify, setNotify] = useState(false);

  const runNotification = () =>{
    setNotify(true)
    setTimeout(() => {
      console.log("hello")
      setNotify(false)
      
    }, 4000);
  }


  const handleAlert = async () => {
    setMsg("Alert");
    try {
      const resp = await axios.post(`https://glacial-lake-outburst-floods-system.onrender.com/api/alert/${msg}`);
      console.log("Resp = ", resp.data.msg);
      if(resp.data.msg === "Done"){
        runNotification();
      }
    } catch (error) {
      console.log("Error ", error);
    }
  }
  
  return (
    <div>
      <div className='bg-slate-900 text-white font-light flex flex-col justify-center items-center h-screen '>
      {notify && (
        <div className="bg-blue-100 border-t border-b border-blue-500 text-blue-700 px-4 py-3 absolute z-10 mb-96 animate-pulse" role="alert">
          <p className="font-bold">Alert Sent</p>
          <p className="text-sm">It{"'"}ll take some moments to reach to people</p>
        </div>
        )}
        <button onClick={() => navigate("/weatherData")} className='bg-slate-700 py-2 px-4 my-2 text-lg rounded-lg w-[20%]'>check Real-Time Wheather</button>
        <button onClick={() => navigate("/temp-humidity")} className='bg-slate-700 py-2 px-4 my-2 text-lg rounded-lg w-[20%]'>Humidity And Temperature</button>
        <button onClick={() => navigate("/airQuality")} className='bg-slate-700 py-2 px-4 my-2 text-lg rounded-lg w-[20%]'>Air Quality</button>
        <button onClick={() => navigate("/alertDisplay")} className='bg-slate-700 py-2 px-4 my-2 text-lg rounded-lg w-[20%]'>Alert Messages</button>
        <button onClick={handleAlert} className='bg-slate-700 py-2 px-4 my-2 text-lg rounded-lg w-[20%]'>Send alert</button>
      </div>
    </div>
  )
}

export default GovMain
