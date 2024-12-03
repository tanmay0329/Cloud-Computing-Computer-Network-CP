const express = require("express");
const cors = require('cors');
const http = require("http");
const axios = require("axios");
const bodyParser = require("body-parser");
const connectDB = require('./connectDB');
const { Server } = require("socket.io");
const mongoose = require('mongoose');
const twilio = require('twilio');
require('dotenv').config();


const app = express();
const server = http.createServer(app);

// CORS for both HTTP requests and Socket.io
app.use(cors({
    origin: 'http://glof-frontend.s3-website.ap-south-1.amazonaws.com', // Allow only your React app's origin
    methods: ['GET', 'POST'],        // Allow specific HTTP methods
    credentials: true                // Allow cookies and other credentials
}));

const io = new Server(server, {
    cors: {
        origin: 'http://glof-frontend.s3-website.ap-south-1.amazonaws.com', // Ensure it's the same as above
        methods: ['GET', 'POST'],
        credentials: true
    }
});

const {
    handleEmergencyRespLogIn,
    handleEmergencyRespValidation,
    handleGenralPublicLogIn,
    handleGenralPublicValidation,
    handleGovernmentAuthLogin,
    handleGovernmentAuthValidation, 
} = require('./controllers/UserData')

const {
    handleAlertForm,
    handleAllAlertForm
} = require('./controllers/Alert');

let allContacts = [];

// Handle Alert Function
async function handleAlert() {
    try {
        const db = mongoose.connection;

        // List of login collection names
        const loginCollections = ['generalpublicloginschemas', 'emergencyrespondersloginschemas', 'governmentauthoritiesloginschemas'];

        for (const collectionName of loginCollections) {
            // Get the collection
            const collection = db.collection(collectionName);

            // Find all documents and project only the 'contact' field
            const contacts = await collection.find({}, { projection: { contact: 1, _id: 0 } }).toArray();

            // Extract contacts and push them to the allContacts array
            allContacts = allContacts.concat(contacts.map(doc => doc.contact));
        }

        console.log('All contacts:', allContacts);

    } catch (error) {
        console.error('Error retrieving contacts:', error);
    }
}

// WebSocket initialization
io.on("connection", (socket) => {
    console.log("Socket.io client connected:", socket.id);

    // Receive message from client
    socket.on("client-message", (message) => {
        console.log("Message from client:", message);

        if (message === "Alert") {
            handleAlertMsg();
        }

        io.emit("message", message); // Broadcast the message to all clients
    });

    // Handle client disconnection
    socket.on("disconnect", () => {
        console.log("Socket.io client disconnected:", socket.id);
    });
});

// Function to handle alert messages (which was missing)
function handleAlertMsg() {
    console.log('Alert message handled');
    // Add your logic here
}

// MongoDB Connection and Server Start
const uri = "mongodb+srv://akhileshpimple3:ytYxoa3so6WZxoxF@cluster-glof.cxbkb.mongodb.net/?retryWrites=true&w=majority&appName=Cluster-glof";
connectDB.connectToDB(uri)
    .then(() => {
        console.log("DB connected");
        handleAlert(); // Only call handleAlert after DB connection
        const PORT = 3000;
        app.listen(PORT, '0.0.0.0', () => {
            console.log(`App running on port ${PORT}`);
        });
    })
    .catch((error) => {
        console.error('Failed to connect to MongoDB', error);
    });

// Middleware for parsing JSON and URL-encoded bodies
app.use(express.json());
app.use(bodyParser.json());
app.use(express.urlencoded({ extended: false }));

// API routes
app.get("/api/msg", (req, res) => {
    res.json({ msg: "hello" });
});

app.get("/api/weatherData/:place", async (req, res) => {
    try {
        // Removed extra curly braces around req.params.place
        const response = await axios.get(`http://api.weatherapi.com/v1/current.json?key=87d09b2a950c4a63a0c83743240409&q=${req.params.place}&aqi=yes`);
        console.log("Weather API response = ", response.data);
        res.json(response.data);
    } catch (err) {
        console.error("Error occurred while fetching weather data: ", err);
        res.status(500).json({ error: "Failed to fetch weather data" });
    }
});

app.post("/api/alert/:msg", async (req, res) => {
    const data = req.params;
    console.log("Alert data = ", data);
    if (data && data.msg === "Alert") {
        console.log("Alert called")
        handleAlertMsg();
        for(call in allContacts){
            console.log("Call - ",call)
            const accountSid = process.env.TWILIO_SID; 
            const authToken = process.env.TWILIO_AUTH;   
    
            const client = new twilio(accountSid, authToken);
    
            client.messages
                .create({
                    body: 'Residents in low-lying areas are urged to stay alert for the risk of a Glacial Lake Outburst Flood (GLOF). Heavy rain or rising temperatures may cause sudden flooding. Evacuate to higher ground immediately and follow official instructions.', 
                    from: '+12342941452',
                    to: `+91${allContacts[call]}`  
                })
                .then(message => console.log(`Message sent with SID: ${message.sid}`))
                .catch(error => console.error('Error sending message:', error));

        }

        // console.log("all fuck = ",allContacts)

        res.json({ msg: "Done" });
    } else {
        res.json({ msg: "NotDone" });
    }
});

app.get("/api/hello", (req, res) => {
    res.json({ msg: "Hello this is akhilesh" });
});

// Handling General Public Login
app.post("/api/general-public-singup/login-data", async (req, res) => {
    try {
        const data = req.body;
        const result = await handleGenralPublicLogIn(data);
        console.log("General Public Login result: ", result);
        res.json({ msg: "done" });
    } catch (error) {
        console.error("Error in General Public Login: ", error);
        res.status(500).json({ error: "Failed to log in" });
    }
});

// Handling General Public Validation
app.post("/api/general-public-validation/data", async (req, res) => {
    try {
        const data = req.body;
        const result = await handleGenralPublicValidation(data);
        console.log("General Public Validation result: ", result);
        if (data.location === result.location) {
            res.json({ msg: "done" });
        }
    } catch (error) {
        console.error("Error in General Public Validation: ", error);
        res.status(500).json({ error: "Validation failed" });
    }
});

// Handling Government Authority Login
app.post("/api/government-auth-singup/login-data", async (req, res) => {
    try {
        const data = req.body;
        const result = await handleGovernmentAuthLogin(data);
        console.log("Government Authority Login result: ", result);
        res.json({ msg: "done" });
    } catch (error) {
        console.error("Error in Government Authority Login: ", error);
        res.status(500).json({ error: "Failed to log in" });
    }
});

// Handling Government Authority Validation
app.post("/api/government-auth-validation/data", async (req, res) => {
    try {
        const data = req.body;
        const result = await handleGovernmentAuthValidation(data);
        console.log("Government Authority Validation result: ", result);
        res.json({ msg: "done" });
    } catch (error) {
        console.error("Error in Government Authority Validation: ", error);
        res.status(500).json({ error: "Validation failed" });
    }
});

// Handling Emergency Responder Login
app.post("/api/emergency-resp-singup/login-data", async (req, res) => {
    try {
        const data = req.body;
        const result = await handleEmergencyRespLogIn(data);
        console.log("Emergency Responder Login result: ", result);
        res.json({ msg: "done" });
    } catch (error) {
        console.error("Error in Emergency Responder Login: ", error);
        res.status(500).json({ error: "Failed to log in" });
    }
});

// Handling Emergency Responder Validation
app.post("/api/emergency-resp-validation/data", async (req, res) => {
    try {
        const data = req.body;
        const result = await handleEmergencyRespValidation(data);
        console.log("Emergency Responder Validation result: ", result);
        res.json({ msg: "done" });
    } catch (error) {
        console.error("Error in Emergency Responder Validation: ", error);
        res.status(500).json({ error: "Validation failed" });
    }
});

app.post("/api/send-alert-form",async(req,res)=>{
    try {
        const data = req.body;
        console.log("data = ",data.name)
        const result = await handleAlertForm(data);
        console.log("Result of submit of alert = ",result);
        
        res.json({msg:"done"})
    } catch (error) {
        console.error("Error in Submitting the alert form: ", error);
        res.status(500).json({ error: "Validation failed" });
    }
})

app.get("/api/all-alert-data", async(req,res)=>{
    try {
        const result = await handleAllAlertForm()
        console.log("All alert fomrs = ",result);
        res.json({alertForm:result});

    } catch (error) {
        console.log("Error while getting alert from data in server ",error)
    }
})

app.get("/api/hello", async (req, res) => {
    res.json({ msg: "Hello from GLOF" })
})
