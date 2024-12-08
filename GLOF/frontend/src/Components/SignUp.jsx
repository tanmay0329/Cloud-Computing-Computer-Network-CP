
import { useState } from 'react'
import axios from 'axios';
import { useNavigate } from 'react-router-dom';

const SignUp = () => {
  const navigate = useNavigate();

  const [fullName,setName] = useState("")
  const [email ,setEmail] = useState("")
  const [contact ,setContact] = useState("")
  const [password ,setPassword] = useState("")  

  const handleSubmitSignUpData = async() =>{
    try{
        const resp = await axios.post("https://glacial-lake-outburst-floods-system.onrender.com/api/general-public-singup/login-data",{
          fullName,
          email,
          contact,
          password,
        })
        console.log(resp);
        if(resp.data.msg==="done"){
          navigate("/genral-public-validation")
        }
    } catch(err){
      console.log("Err = ",err)
    }
  }
  return (
    <div className='flex flex-col justify-center items-center bg-slate-900 text-slate-200 h-screen'>
        <div className='text-4xl mb-10'>Genral Public Sign Up</div>
        <div className='flex flex-col text-lg font-light text-slate-200'>
          <div>Full Name</div>
          <input type="text" placeholder='Enter Name' value={fullName} onChange={(e)=>setName(e.target.value)}className="bg-slate-600 py-2 px-4 mx-5 my-2 rounded-lg"/>
          <div>Email</div>
          <input type="text" placeholder='Enter Email' value={email} onChange={(e)=>setEmail(e.target.value)} className="bg-slate-600 py-2 px-4 mx-5 my-2 rounded-lg"/>
          <div>Contact</div>
          <input type="text" placeholder='Contact Info' value={contact} onChange={(e)=>setContact(e.target.value)} className="bg-slate-600 py-2 px-4 mx-5 my-2 rounded-lg"/>
          <div>Password</div>
          <input type="text" placeholder='Enter Password' value={password} onChange={(e)=>setPassword(e.target.value)} className="bg-slate-600 py-2 px-4 mx-5 my-2 rounded-lg"/>
          <button className='bg-slate-950 py-2 px-4 m-8 my-2 rounded-lg' onClick={handleSubmitSignUpData}>Submit</button>
        </div>
    </div>
  )
}

export default SignUp
