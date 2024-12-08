
import { useState } from 'react'
import axios from 'axios';
import { useNavigate } from 'react-router-dom';

const SignUp = () => {
  const navigate = useNavigate();

  const [jurisdiction,setJurisdiction] = useState("")
  const [emergencyContactInformation ,setEmergencyContactInformation] = useState("")
  const [accessLevel,setAccessLevel] = useState("")
  const [governementIdProof ,setGovernemntIdProof] = useState("")
  const [governementIdNumber ,setGovernementIdNumber] = useState("")  

  const handleSubmitSignUpData = async() =>{
    try{
        const resp = await axios.post("https://glacial-lake-outburst-floods-system.onrender.com/api/government-auth-validation/data",{
          jurisdiction,
          emergencyContactInformation,
          accessLevel,
          governementIdProof,
          governementIdNumber,
        })
        console.log(resp);
        if(resp.data.msg==="done"){
          navigate("/main")
        }
    } catch(err){
      console.log("Err = ",err)
    }
  }
  return (
    <div className='flex flex-col justify-center items-center bg-slate-900 text-slate-200 h-screen'>
        <div className='text-4xl mb-10'>Genral Public Validation</div>
        <div className='flex flex-col text-lg font-light text-slate-200'>
          <div>Orgnization</div>
          <input type="text" placeholder='Enter Name' value={jurisdiction} onChange={(e)=>setJurisdiction(e.target.value)}className="bg-slate-600 py-2 px-4 mx-5 my-2 rounded-lg"/>
          <div>Area Of Operation</div>
          <input type="text" placeholder='Enter Name' value={emergencyContactInformation} onChange={(e)=>setEmergencyContactInformation(e.target.value)}className="bg-slate-600 py-2 px-4 mx-5 my-2 rounded-lg"/>
          <div>Access Level</div>
          <input type="text" placeholder='Enter Email' value={accessLevel} onChange={(e)=>setAccessLevel(e.target.value)} className="bg-slate-600 py-2 px-4 mx-5 my-2 rounded-lg"/>
          <div>Governemtn Id Proof</div>
          <input type="text" placeholder='Contact Info' value={governementIdProof} onChange={(e)=>setGovernemntIdProof(e.target.value)} className="bg-slate-600 py-2 px-4 mx-5 my-2 rounded-lg"/>
          <div>Government Id Number</div>
          <input type="text" placeholder='Enter Password' value={governementIdNumber} onChange={(e)=>setGovernementIdNumber(e.target.value)} className="bg-slate-600 py-2 px-4 mx-5 my-2 rounded-lg"/>
          <button className='bg-slate-950 py-2 px-4 m-8 my-2 rounded-lg' onClick={handleSubmitSignUpData}>Submit</button>
        </div>
    </div>
  )
}

export default SignUp
