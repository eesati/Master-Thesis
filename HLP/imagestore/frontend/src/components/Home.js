import react, { useState, useEffect } from "react";
import { Button, Grid, TextField, Card } from "@material-ui/core";
import Paper from "@material-ui/core/Paper";
import { makeStyles } from "@material-ui/core/styles";
import axios from "axios";
const useStyles = makeStyles((theme) => ({
  root: {
    display: "flex",
    flexWrap: "wrap",
    "& > *": {
      margin: theme.spacing(1),
      width: theme.spacing(48),
      height: theme.spacing(48),
    },
  },
}));
var prefix = "data:image/png;base64,";
let Home = (props) => {
  const classes = useStyles();
  const [image, setImage] = useState(null);
  const [fileHash, setFileHash] = useState("");
  let getImage = () => {
    console.log(fileHash);
    if (fileHash === "") {
    } else {
      axios
        .get("http://localhost:8585/getfile?filehash=" + fileHash)
        .then((response) => {
          setImage(response.data);
          console.log(response.data);
        })
        .catch((error) => {
          console.log(error);
        });
    }
  };
  return (
    <Grid
      container
      direction="column"
      justify="center"
      alignItems="center"
      spacing={3}
    >
      <Grid item>
        {" "}
        <TextField
          id="outlined-basic"
          label="Face id"
          variant="outlined"
          value={fileHash}
          onChange={(e) => {
            setFileHash(e.target.value);
          }}
        />
      </Grid>
      <Grid item>
        {" "}
        <Button variant="contained" color="primary" onClick={getImage}>
          Get Image
        </Button>{" "}
      </Grid>
      <Grid item>
        <div className={classes.root}>
          <Paper variant="outlined" square elevation={3}>
            {image ? (
              <img style={{ width: "100%" }} src={prefix + image}></img>
            ) : (
                //   <img src={prefix + image}></img>
                <label>Image will be displayed here...</label>
              )}
          </Paper>
        </div>
      </Grid>
    </Grid>
  );
};

export default Home;
